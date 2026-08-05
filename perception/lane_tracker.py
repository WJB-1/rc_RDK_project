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
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.logger import get_logger

from vision.models.bisenet import SegmentationEngine
from vision.algorithms.ipm import MathematicalIPM
from vision.algorithms.lane_analyzer import analyze_bev_lane_state, get_last_valid_state
from vision.algorithms.mask_utils import clean_mask_by_cc
from perception.ipm_drawer import draw_debug_panel
import numpy as np
import cv2


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
        seg_cfg = settings.get('segmentation', {})
        model_path = seg_cfg.get('model_path', 'models/bisenetv2_lane_x5.bin')
        # 将相对路径转换为基于项目根目录的绝对路径，避免受 CWD 影响
        if not Path(model_path).is_absolute():
            project_root = Path(__file__).parent.parent
            model_path = str(project_root / model_path)
        input_size = seg_cfg.get('input_size', 512)
        target_class = seg_cfg.get('target_class', 1)

        self.seg_engine = SegmentationEngine(
            model_path=model_path,
            input_size=input_size,
            target_class=target_class,
        )
        self.logger.info(f"SegmentationEngine 初始化完成: {model_path}")

        # ------------------------------------------------------------------
        # 2) Math IPM 引擎
        # ------------------------------------------------------------------
        ipm_cfg = settings.get('math_ipm', {})
        self.ipm = MathematicalIPM(
            img_w=ipm_cfg.get('img_w', 1920),
            img_h=ipm_cfg.get('img_h', 1080),
            focal_length_mm=ipm_cfg.get('focal_length_mm', 2.8),
            pixel_size_mm=ipm_cfg.get('pixel_size_mm', 0.003),
            camera_height_mm=ipm_cfg.get('camera_height_mm', 190.0),
            pitch_deg=ipm_cfg.get('pitch_deg', 40.0),
            canvas_w=ipm_cfg.get('canvas_w', 400),
            canvas_h=ipm_cfg.get('canvas_h', 400),
            pixel_per_mm=ipm_cfg.get('pixel_per_mm', 0.5),
            blind_spot_mm=ipm_cfg.get('blind_spot_mm', 200.0),
        )
        self.physical_track_width_mm = ipm_cfg.get('physical_track_width_mm', 450.0)
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

            # 1) BPU 推理 -> 掩码
            _t1 = _t.time()
            mask = self.seg_engine.inference(frame)

            # 2) 连通域清洗
            clean_mask, noise_mask = clean_mask_by_cc(mask, min_bottom_y=mask.shape[0] - 10)

            # 3) 逆透视变换
            bev_mask = self.ipm.warp(clean_mask)

            # 4) 车道状态分析
            lane_state = analyze_bev_lane_state(
                bev_mask=bev_mask,
                ipm_engine=self.ipm,
                blind_spot_px=self.ipm.blind_spot_px,
                track_width_mm=self.physical_track_width_mm,
            )

            if lane_state.get("frame_dropped", False):
                lane_state = get_last_valid_state()

            self.last_lane_state = lane_state
            self.last_seg_mask = clean_mask

            offset_mm = lane_state['pid_error_mm']
            is_intersection = lane_state['crossroad_detected']

            # 5) 渲染 debug 面板
            debug_frame = draw_debug_panel(
                clean_mask=clean_mask, bev_mask=bev_mask,
                lane_state=lane_state, raw_image=frame,
                camera_pitch_deg=self.ipm.pitch_deg,
                physical_track_width_mm=self.physical_track_width_mm,
                noise_mask=noise_mask,
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
