# -*- coding: utf-8 -*-

"""
车道线追踪器 - LaneTracker (Mathematical IPM 版)

职责:
- 基于 BPU 语义分割模型提取赛道掩码
- 通过 Mathematical IPM 生成物理尺度 BEV
- 输出毫米级横向偏移 offset_mm 与路口状态 is_intersection
- 提供四宫格 Debug 面板（供本地或 C/S 可视化使用）

输出变更说明:
- 旧版: process(frame) -> (offset: float [-1,1], debug_frame)
- 新版: process(frame) -> (offset_mm: float, is_intersection: bool, debug_frame)
"""

import time
from typing import Tuple, Optional
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

import sys
sys.path.insert(0, str(Path(__file__).parents[2]))
from utils.logger import get_logger

from perception.models.pidinet import PiDiNetEngine
from perception.models.bisenet import SegmentationEngine
from perception.algorithms.ipm import MathematicalIPM
from perception.algorithms.pidinet_lane import (
    GroundIPMLanePairSelector,
    TemplateDistanceLaneSelector,
    lines_to_mask,
)
from perception.algorithms.mask_utils import clean_mask_by_cc
from perception.diagnostics.ipm_drawer import draw_debug_panel
import numpy as np
import cv2


def build_undistort_parameters(calibration: dict, frame_shape):
    """Build an input-resolution camera matrix and distortion vector."""
    if not calibration or not calibration.get("enabled", False):
        return None, None
    height, width = frame_shape[:2]
    source_width = float(calibration.get("image_width", width))
    source_height = float(calibration.get("image_height", height))
    if source_width <= 0 or source_height <= 0:
        raise ValueError("camera calibration image dimensions must be positive")
    scale_x = width / source_width
    scale_y = height / source_height
    camera_matrix = np.array(
        [
            [float(calibration["fx_px"]) * scale_x, 0.0, float(calibration["cx_px"]) * scale_x],
            [0.0, float(calibration["fy_px"]) * scale_y, float(calibration["cy_px"]) * scale_y],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
    distortion = np.asarray(calibration.get("distortion", [0, 0, 0, 0, 0]), dtype=np.float64).reshape(-1)
    if distortion.size < 4:
        raise ValueError("camera calibration distortion requires at least k1,k2,p1,p2")
    if distortion.size < 5:
        distortion = np.pad(distortion, (0, 5 - distortion.size))
    return camera_matrix, distortion[:5]


class LaneTracker:
    """
    基于语义分割 + 数学逆透视的车道线追踪器
    """

    def __init__(self, settings: dict):
        """
        初始化巡线追踪器

        Args:
            settings: 配置字典，需包含 segmentation 与 math_ipm 块
        """
        self.logger = get_logger()

        if cv2 is None or np is None:
            raise ImportError("OpenCV 和 NumPy 是必需的依赖")

        # ------------------------------------------------------------------
        # 1) BPU 分割引擎
        # ------------------------------------------------------------------
        edge_cfg = settings.get('edge_detection', {})
        model_path = edge_cfg.get('model_path', 'models/pidinet_small_dil_512x384_x5_bayese.bin')
        # 将相对路径转换为基于项目根目录的绝对路径，避免受 CWD 影响
        if not Path(model_path).is_absolute():
            project_root = Path(__file__).parent.parent
            model_path = str(project_root / model_path)
        input_width = int(edge_cfg.get('input_width', 512))
        input_height = int(edge_cfg.get('input_height', 384))

        self.edge_engine = PiDiNetEngine(
            model_path=model_path,
            input_size=(input_width, input_height),
            edge_threshold=float(edge_cfg.get('threshold', 0.25)),
            angle_tol_deg=float(edge_cfg.get('angle_tol_deg', 3.0)),
            normal_dist_tol=float(edge_cfg.get('normal_dist_tol', 8.0)),
        )
        self.logger.info(f"PiDiNetEngine initialized: {model_path}")

        semantic_cfg = edge_cfg.get('semantic_gate', {})
        self.semantic_engine = None
        self.semantic_gate_enabled = bool(semantic_cfg.get('enabled', False))
        if bool(semantic_cfg.get('available', True)):
            semantic_model_path = semantic_cfg.get('model_path', 'models/bisenetv2_lane_x5.bin')
            if not Path(semantic_model_path).is_absolute():
                semantic_model_path = str(Path(__file__).parent.parent / semantic_model_path)
            try:
                self.semantic_engine = SegmentationEngine(
                    model_path=semantic_model_path,
                    input_size=int(semantic_cfg.get('input_size', 512)),
                    target_class=int(semantic_cfg.get('target_class', 1)),
                )
                self.logger.info(f"Semantic gate initialized: {semantic_model_path}")
            except Exception as error:
                self.logger.warning(f"Semantic gate disabled: {error}")
        self.semantic_gate_enabled = self.semantic_gate_enabled and self.semantic_engine is not None

        # ------------------------------------------------------------------
        # 2) Math IPM 引擎
        # ------------------------------------------------------------------
        ipm_cfg = settings.get('math_ipm', {})
        calibration_cfg = dict(ipm_cfg.get('camera_calibration', {}) or {})
        if not calibration_cfg:
            calibration_cfg = {"enabled": False}
        self.camera_calibration = calibration_cfg
        self.ipm = MathematicalIPM(
            img_w=ipm_cfg.get('img_w', 1920),
            img_h=ipm_cfg.get('img_h', 1080),
            focal_length_mm=ipm_cfg.get('focal_length_mm', 2.8),
            pixel_size_mm=ipm_cfg.get('pixel_size_mm', 0.003),
            fx_px=ipm_cfg.get('fx_px'),
            fy_px=ipm_cfg.get('fy_px'),
            cx_px=ipm_cfg.get('cx_px'),
            cy_px=ipm_cfg.get('cy_px'),
            camera_height_mm=ipm_cfg.get('camera_height_mm', 190.0),
            pitch_deg=ipm_cfg.get('pitch_deg', 40.0),
            canvas_w=ipm_cfg.get('canvas_w', 400),
            canvas_h=ipm_cfg.get('canvas_h', 400),
            pixel_per_mm=ipm_cfg.get('pixel_per_mm', 0.5),
            blind_spot_mm=ipm_cfg.get('blind_spot_mm', 200.0),
        )
        track_cfg = settings.get('track', {})
        self.physical_track_width_mm = float(track_cfg.get('lane_width_mm', 200.0))
        vehicle_geometry = settings.get('vehicle_geometry', {})
        selector_class = (TemplateDistanceLaneSelector
                          if edge_cfg.get('pair_selector', 'template_distance') == 'template_distance'
                          else GroundIPMLanePairSelector)
        self.lane_selector = selector_class(
            input_size=(input_width, input_height),
            calibration_size=(ipm_cfg.get('img_w', 1920), ipm_cfg.get('img_h', 1080)),
            fx_px=ipm_cfg.get('fx_px', 1131.5665939049366),
            fy_px=ipm_cfg.get('fy_px', 1131.5665939049366),
            cx_px=ipm_cfg.get('cx_px', 951.0970902161366),
            cy_px=ipm_cfg.get('cy_px', 553.6989838702775),
            camera_height_mm=ipm_cfg.get('camera_height_mm', 163.52111350206778),
            pitch_deg=ipm_cfg.get('pitch_deg', 35.41572788367764),
            canvas_w=ipm_cfg.get('canvas_w', 400),
            canvas_h=ipm_cfg.get('canvas_h', 400),
            pixel_per_mm=ipm_cfg.get('pixel_per_mm', 0.5),
            blind_spot_mm=ipm_cfg.get('blind_spot_mm', 90.0),
            lane_width_mm=self.physical_track_width_mm,
            vehicle_geometry=vehicle_geometry,
            pair_profiles=edge_cfg.get('pair_profiles'),
            debug_pair_profile=edge_cfg.get('debug_pair_profile', 'auto'),
            semantic_gate=self.semantic_gate_enabled,
            semantic_min_coverage=float(semantic_cfg.get('min_coverage', 0.01)),
            semantic_line_thickness=int(semantic_cfg.get('line_thickness', 3)),
        )
        self.last_bev_mask = None
        self.last_original_view = None
        self.last_lane_state = None  # 供 main.py 读取最新一帧的完整状态
        self.last_seg_mask = None     # 供 main.py 的最新分割mask，用于seg-based路口检测
        self.logger.info(
            f"MathematicalIPM 初始化完成 (三段式状态机): "
            f"pitch={self.ipm.pitch_deg}°, height={self.ipm.camera_height_mm}mm, "
            f"canvas={self.ipm.canvas_size}, blind_spot={self.ipm.blind_spot_mm}mm"
        )

        # ------------------------------------------------------------------
        # 3) 调试选项
        # ------------------------------------------------------------------
        # show_video 控制本地 cv2.imshow 窗口（已废弃），
        # 但 debug_panel 始终生成供 Web 调试服务器使用
        self.debug = settings.get('debug', {}).get('show_video', True)
        self.logger.info("LaneTracker 初始化完成 (Mathematical IPM 模式)")

    def set_semantic_gate(self, enabled: bool) -> bool:
        """Enable or disable semantic candidate gating for subsequent frames."""
        self.semantic_gate_enabled = bool(enabled) and self.semantic_engine is not None
        self.lane_selector.semantic_gate = self.semantic_gate_enabled
        return self.semantic_gate_enabled

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        camera_matrix, distortion = build_undistort_parameters(self.camera_calibration, frame.shape)
        if camera_matrix is None:
            return frame
        return cv2.undistort(frame, camera_matrix, distortion, camera_matrix)

    def process(self, frame: Optional[np.ndarray]) -> Tuple[float, bool, np.ndarray]:
        """
        处理单帧图像，输出物理坐标系误差与路口状态

        Args:
            frame: BGR 格式的 numpy 数组

        Returns:
            Tuple[float, bool, np.ndarray]:
                - offset_mm: 横向误差 (mm)，负值表示车体偏左需向右修正
                - is_intersection: 是否检测到路口
                - debug_frame: 四宫格调试面板或原始帧副本
        """
        start_time = time.time()

        # 防御性检查
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            self.logger.warning("输入帧无效")
            return 0.0, False, np.zeros((240, 320, 3), dtype=np.uint8)

        try:
            import time as _t

            processing_input = self._undistort(frame)

            # 1) BPU 推理 -> 掩码
            _t1 = _t.time()
            edge_mask = self.edge_engine.inference(processing_input)
            semantic_mask = None
            if self.semantic_gate_enabled and self.semantic_engine is not None:
                semantic_raw = self.semantic_engine.inference(processing_input)
                semantic_mask, _ = clean_mask_by_cc(semantic_raw, min_bottom_y=semantic_raw.shape[0] - 10)
            source_lines = self.edge_engine.last_lines
            lane_state = self.lane_selector.analyze(source_lines, semantic_mask=semantic_mask)
            selected_lines = self.lane_selector.detected_source_lines
            line_mask = lines_to_mask(self.edge_engine.last_lines, edge_mask.shape, thickness=3)
            clean_mask = lines_to_mask(selected_lines, edge_mask.shape, thickness=4)
            if not np.any(clean_mask):
                clean_mask = line_mask
            noise_mask = cv2.bitwise_and(edge_mask, cv2.bitwise_not(line_mask))

            # 2) 连通域清洗

            # 3) 逆透视变换
            bev_mask = self.lane_selector.draw_bev_view()
            processing_frame = cv2.resize(
                processing_input, (self.edge_engine.input_width, self.edge_engine.input_height), interpolation=cv2.INTER_AREA
            )
            original_view = self.lane_selector.draw_original_view(processing_frame)

            # 4) 车道状态分析
            lane_state["raw_line_count"] = len(self.edge_engine.last_raw_lines)
            lane_state["line_count"] = len(self.edge_engine.last_lines)
            lane_state["line_method"] = (
                "HoughLinesP + PCA merge + six-line template distance"
                if isinstance(self.lane_selector, TemplateDistanceLaneSelector)
                else "HoughLinesP + PCA merge (a3d8)"
            )


            self.last_lane_state = lane_state
            self.last_seg_mask = semantic_mask if semantic_mask is not None else clean_mask
            self.last_bev_mask = bev_mask
            self.last_original_view = original_view

            offset_mm = lane_state['pid_error_mm']
            is_intersection = lane_state['crossroad_detected']

            # 5) 渲染 debug 面板
            debug_frame = draw_debug_panel(
                clean_mask=clean_mask, bev_mask=bev_mask,
                lane_state=lane_state,
                raw_image=original_view,
                camera_pitch_deg=self.ipm.pitch_deg,
                physical_track_width_mm=self.physical_track_width_mm,
                noise_mask=noise_mask,
                semantic_mask=semantic_mask,
            )

            process_time = (time.time() - start_time) * 1000
            if process_time > 10:
                self.logger.warning(f"巡线处理耗时: {process_time:.0f}ms")

            return offset_mm, is_intersection, debug_frame

        except Exception as e:
            self.logger.exception(f"巡线处理异常: {e}")
            return 0.0, False, frame.copy() if frame is not None else np.zeros((240, 320, 3), dtype=np.uint8)

    def reset(self):
        """重置追踪器状态"""
        self.logger.info("LaneTracker 已重置")
