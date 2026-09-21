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

from perception.algorithms.timing import reset_frame, block, get_frame_timings
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
            settings: 配置字典；优先读取 ``lane_detection`` 标签，
                      缺失时回退到 ``math_ipm`` / ``edge_detection`` / ``track``。
        """
        self.logger = get_logger()

        if cv2 is None or np is None:
            raise ImportError("OpenCV 和 NumPy 是必需的依赖")

        # ==============================================================
        # 0) 配置读取：优先 lane_detection，缺失时回退旧标签
        # ==============================================================
        lane_cfg = dict(settings.get('lane_detection', {}) or {})
        legacy_ipm_cfg = dict(settings.get('math_ipm', {}) or {})
        legacy_edge_cfg = dict(settings.get('edge_detection', {}) or {})
        legacy_track_cfg = dict(settings.get('track', {}) or {})
        legacy_vehicle_geometry = dict(settings.get('vehicle_geometry', {}) or {})

        # --- 去畸变：优先 lane_detection.undistort_camera ---
        self.camera_calibration = dict(lane_cfg.get('undistort_camera', {}) or {})
        if not self.camera_calibration:
            self.camera_calibration = dict(
                legacy_ipm_cfg.get('camera_calibration', {}) or {}
            )
        if not self.camera_calibration:
            self.camera_calibration = {"enabled": False}

        # --- IPM 相机内外参：优先 lane_detection.ipm_camera ---
        self._undistort_maps = None
        ipm_camera_cfg = dict(lane_cfg.get('ipm_camera', {}) or {})

        def _pick(key, default=None, legacy_key=None):
            if key in ipm_camera_cfg:
                return ipm_camera_cfg[key]
            lk = legacy_key if legacy_key is not None else key
            if lk in legacy_ipm_cfg:
                return legacy_ipm_cfg[lk]
            return default

        img_w = int(_pick('img_w', default=1920))
        img_h = int(_pick('img_h', default=1080))
        fx_px = _pick('fx_px', default=1131.5665939049366)
        fy_px = _pick('fy_px', default=1131.5665939049366)
        cx_px = _pick('cx_px', default=951.0970902161366)
        cy_px = _pick('cy_px', default=553.6989838702775)
        camera_height_mm = _pick('camera_height_mm', default=190.0)
        pitch_deg = _pick('pitch_deg', default=40.0)
        canvas_w = int(_pick('canvas_w', default=400))
        canvas_h = int(_pick('canvas_h', default=400))
        pixel_per_mm = _pick('pixel_per_mm', default=0.5)
        blind_spot_mm = _pick('blind_spot_mm', default=200.0)
        focal_length_mm = _pick('focal_length_mm', default=2.8)
        pixel_size_mm = _pick('pixel_size_mm', default=0.003)

        # --- 模型输入尺寸：优先 lane_detection.input_size ---
        input_size = lane_cfg.get('input_size', None)
        if input_size is None:
            input_width = int(legacy_edge_cfg.get('input_width', 512))
            input_height = int(legacy_edge_cfg.get('input_height', 384))
        else:
            input_width, input_height = int(input_size[0]), int(input_size[1])

        # --- 模板与权重 ---
        self.template_x_at_ref_mm = dict(
            lane_cfg.get('template_x_at_ref_mm', {}) or {}
        )
        self.identity_weights = dict(
            lane_cfg.get('identity_weights', {}) or {}
        )

        # --- 机器人几何 ---
        self.robot_geometry = dict(
            lane_cfg.get('robot_geometry', legacy_vehicle_geometry) or {}
        )

        # --- 车道宽度 ---
        self.physical_track_width_mm = float(
            lane_cfg.get('lane_width_mm',
                         legacy_track_cfg.get('lane_width_mm', 200.0))
        )

        # --- 匹配/置信度/BEV 参数（供后续 matcher 使用） ---
        self.merge_cfg = dict(lane_cfg.get('merge', {}) or {})
        self.hough_cfg = dict(lane_cfg.get('hough', {}) or {})
        self.edge_alg_cfg = dict(lane_cfg.get('edge', {}) or {})
        self.matching_cfg = dict(lane_cfg.get('matching', {}) or {})
        self.confidence_cfg = dict(lane_cfg.get('confidence', {}) or {})
        self.bev_cfg = dict(lane_cfg.get('bev', {}) or {})

        # ==============================================================
        # 1) BPU 分割引擎
        # ==============================================================
        model_path = legacy_edge_cfg.get(
            'model_path', 'models/pidinet_small_dil_512x384_x5_bayese.bin'
        )
        if not Path(model_path).is_absolute():
            project_root = Path(__file__).parents[2]
            model_path = str(project_root / model_path)

        self.edge_engine = PiDiNetEngine(
            model_path=model_path,
            input_size=(input_width, input_height),
            edge_threshold=float(legacy_edge_cfg.get('threshold', 0.25)),
            angle_tol_deg=float(legacy_edge_cfg.get('angle_tol_deg', 3.0)),
            normal_dist_tol=float(legacy_edge_cfg.get('normal_dist_tol', 8.0)),
        )
        self.logger.info(f"PiDiNetEngine initialized: {model_path}")

        semantic_cfg = legacy_edge_cfg.get('semantic_gate', {})
        self.semantic_engine = None
        self.semantic_gate_enabled = bool(semantic_cfg.get('enabled', False))
        if bool(semantic_cfg.get('available', True)):
            semantic_model_path = semantic_cfg.get(
                'model_path', 'models/bisenetv2_lane_x5.bin'
            )
            if not Path(semantic_model_path).is_absolute():
                semantic_model_path = str(
                    Path(__file__).parents[2] / semantic_model_path
                )
            try:
                self.semantic_engine = SegmentationEngine(
                    model_path=semantic_model_path,
                    input_size=int(semantic_cfg.get('input_size', 512)),
                    target_class=int(semantic_cfg.get('target_class', 1)),
                )
                self.logger.info(f"Semantic gate initialized: {semantic_model_path}")
            except Exception as error:
                self.logger.warning(f"Semantic gate disabled: {error}")
        self.semantic_gate_enabled = (
            self.semantic_gate_enabled and self.semantic_engine is not None
        )

        # ==============================================================
        # 2) Math IPM 引擎（使用 lane_detection.ipm_camera）
        # ==============================================================
        self.ipm = MathematicalIPM(
            img_w=img_w,
            img_h=img_h,
            focal_length_mm=focal_length_mm,
            pixel_size_mm=pixel_size_mm,
            fx_px=fx_px,
            fy_px=fy_px,
            cx_px=cx_px,
            cy_px=cy_px,
            camera_height_mm=camera_height_mm,
            pitch_deg=pitch_deg,
            canvas_w=canvas_w,
            canvas_h=canvas_h,
            pixel_per_mm=pixel_per_mm,
            blind_spot_mm=blind_spot_mm,
        )

        selector_class = (
            TemplateDistanceLaneSelector
            if legacy_edge_cfg.get('pair_selector', 'template_distance')
            == 'template_distance'
            else GroundIPMLanePairSelector
        )
        self.lane_selector = selector_class(
            input_size=(input_width, input_height),
            calibration_size=(img_w, img_h),
            fx_px=fx_px,
            fy_px=fy_px,
            cx_px=cx_px,
            cy_px=cy_px,
            camera_height_mm=camera_height_mm,
            pitch_deg=pitch_deg,
            canvas_w=canvas_w,
            canvas_h=canvas_h,
            pixel_per_mm=pixel_per_mm,
            blind_spot_mm=blind_spot_mm,
            lane_width_mm=self.physical_track_width_mm,
            vehicle_geometry=self.robot_geometry,
            pair_profiles=legacy_edge_cfg.get('pair_profiles'),
            debug_pair_profile=legacy_edge_cfg.get('debug_pair_profile', 'auto'),
            semantic_gate=self.semantic_gate_enabled,
            semantic_min_coverage=float(semantic_cfg.get('min_coverage', 0.01)),
            semantic_line_thickness=int(semantic_cfg.get('line_thickness', 3)),
        )

        if self.template_x_at_ref_mm:
            self.lane_selector.template_x_at_ref_mm = dict(self.template_x_at_ref_mm)
        if self.identity_weights:
            self.lane_selector.identity_weights = dict(self.identity_weights)

        self.last_bev_mask = None
        self.last_original_view = None
        self.last_lane_state = None
        self.last_seg_mask = None
        self.last_timing = {}

        # ---------------------------------------------------------------
        # Debug 可视化
        # ---------------------------------------------------------------
        self.last_debug_views = {}      # name -> BGR image
        self.last_debug_capture = {}    # 原始中间数据（供 Web 端按需重绘）
        self.debug_dump_dir = None      # 可选：每帧写盘目录
        self._debug_frame_idx = 0
        self.logger.info(
            f"MathematicalIPM 初始化完成 (lane_detection 配置): "
            f"pitch={self.ipm.pitch_deg}°, height={self.ipm.camera_height_mm}mm, "
            f"canvas={self.ipm.canvas_size}, blind_spot={self.ipm.blind_spot_mm}mm"
        )

        # ------------------------------------------------------------------
        # 3) 调试选项
        # ------------------------------------------------------------------
        self.debug = settings.get('debug', {}).get('show_video', True)
        self.logger.info("LaneTracker 初始化完成 (Mathematical IPM 模式)")

        # --- 阶段耗时日志 ---
        debug_cfg = settings.get('debug', {}) or {}
        self.timing_enabled = bool(debug_cfg.get('timing_enabled', True))
        self.timing_log_path = None
        if self.timing_enabled:
            log_path = debug_cfg.get('timing_log_path', 'logs/lane_timing.log')
            if not Path(log_path).is_absolute():
                log_path = str(Path(__file__).parents[2] / log_path)
            self.timing_log_path = log_path
            Path(self.timing_log_path).parent.mkdir(parents=True, exist_ok=True)
            self.logger.info(f"Stage timing log: {self.timing_log_path}")
        self._timing_frame_counter = 0

    def set_semantic_gate(self, enabled: bool) -> bool:
        """Enable or disable semantic candidate gating for subsequent frames."""
        self.semantic_gate_enabled = bool(enabled) and self.semantic_engine is not None
        self.lane_selector.semantic_gate = self.semantic_gate_enabled
        return self.semantic_gate_enabled

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        if not self.camera_calibration or not self.camera_calibration.get("enabled", False):
            return frame
        h, w = frame.shape[:2]
        cache = self._undistort_maps
        if cache is None or cache[2] != (w, h):
            camera_matrix, distortion = build_undistort_parameters(
                self.camera_calibration, frame.shape
            )
            if camera_matrix is None:
                self._undistort_maps = (None, None, (w, h))
            else:
                map1, map2 = cv2.initUndistortRectifyMap(
                    camera_matrix, distortion, None, camera_matrix,
                    (w, h), cv2.CV_16SC2,
                )
                self._undistort_maps = (map1, map2, (w, h))
            cache = self._undistort_maps
        map1, map2, _ = cache
        if map1 is None:
            return frame
        return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

    def _capture_debug(
        self,
        processing_input,
        lane_state,
        clean_mask,
        bev_mask,
        original_view,
        ):
        """收集中间结果，供 Web 端按需渲染 debug 视图。"""
        edge_engine = self.edge_engine
        lane_selector = self.lane_selector

        capture = {
            # 像素域
            "enhanced": getattr(edge_engine, "last_enhanced", None),
            "edge_prob": getattr(edge_engine, "last_probability", None),
            "binary": getattr(edge_engine, "last_binary", None),
            "closed": getattr(edge_engine, "last_closed", None),
            "labels": getattr(edge_engine, "last_label_map", None),
            "n_labels": int(getattr(edge_engine, "last_n_labels", 0)),
            "skeleton": getattr(edge_engine, "last_skeleton", None),
            "hough_lines": list(getattr(edge_engine, "last_raw_lines", []) or []),

            # mask
            "clean_mask": clean_mask,
            "bev_mask": bev_mask,
            "original_view": original_view,

            # selector 内部
            "ground_segments": list(getattr(lane_selector, "ground_segments", []) or []),
            "merged": list(getattr(lane_selector, "merged_segments", []) or []),
            "candidates": dict(getattr(lane_selector, "candidate_slots", {}) or {}),
            "template": dict(self.template_x_at_ref_mm or {}),
            "candidate_radius_mm": float(
                self.matching_cfg.get("candidate_radius_first_frame_mm", 40.0)
            ),
            "y_ref_mm": float(self.matching_cfg.get("y_ref_mm", 500.0)),

            # 最终结果
            "result": getattr(lane_selector, "last_match_result", None),
            "lane_state": dict(lane_state or {}),

            # overlay 需要
            "original_input": processing_input,
            "ground_to_pixel": getattr(self.ipm, "ground_to_pixel", None),
            "scale_up_x": 1.0,
            "scale_up_y": 1.0,

            # BEV 画布
            "canvas": getattr(lane_selector, "bev_canvas", None),
        }

        if capture["canvas"] is None:
            from perception.diagnostics.lane_debug_viz import BevCanvas
            capture["canvas"] = BevCanvas(
                x_min=self.bev_cfg.get("x_min", -600.0),
                x_max=self.bev_cfg.get("x_max", 600.0),
                y_min=self.bev_cfg.get("y_min", -250.0),
                y_max=self.bev_cfg.get("y_max", 800.0),
                ppm=self.bev_cfg.get("ppm", 0.9),
            )

        # -------------------------------------------------------------
        # 新地面系可视化（不动参数，只做展示）
        # 依赖 TemplateDistanceLaneSelector 上的两个 render 方法
        # -------------------------------------------------------------
        if isinstance(lane_selector, TemplateDistanceLaneSelector):
            try:
                capture["new_ground_bev"] = lane_selector.render_new_ground_bev()
            except Exception as error:
                self.logger.warning(f"render_new_ground_bev failed: {error}")
                capture["new_ground_bev"] = None

            try:
                capture["new_ground_original"] = lane_selector.render_original_with_ground_centerline(
                    processing_input
                )
            except Exception as error:
                self.logger.warning(
                    f"render_original_with_ground_centerline failed: {error}"
                )
                capture["new_ground_original"] = None
        else:
            capture["new_ground_bev"] = None
            capture["new_ground_original"] = None

        self.last_debug_capture = capture
        return capture

    def process(self, frame: Optional[np.ndarray]) -> Tuple[float, bool, np.ndarray]:
        start_time = time.time()
        reset_frame()
        self._timing_frame_counter += 1

        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            self.logger.warning("输入帧无效")
            return 0.0, False, np.zeros((240, 320, 3), dtype=np.uint8)

        try:
            with block("tracker.undistort"):
                processing_input = self._undistort(frame)

            inference_started = time.perf_counter()
            with block("tracker.edge_inference"):
                edge_mask = self.edge_engine.inference(processing_input)
            inference_ms = (time.perf_counter() - inference_started) * 1000.0

            semantic_mask = None
            if self.semantic_gate_enabled and self.semantic_engine is not None:
                with block("tracker.semantic_inference"):
                    semantic_raw = self.semantic_engine.inference(processing_input)
                    semantic_mask, _ = clean_mask_by_cc(
                        semantic_raw, min_bottom_y=semantic_raw.shape[0] - 10
                    )

            source_lines = self.edge_engine.last_lines
            fitting_started = time.perf_counter()
            with block("tracker.selector_analyze"):
                lane_state = self.lane_selector.analyze(source_lines, semantic_mask=semantic_mask)
            fitting_ms = (time.perf_counter() - fitting_started) * 1000.0

            selected_lines = self.lane_selector.detected_source_lines
            with block("tracker.lines_to_mask"):
                line_mask = lines_to_mask(self.edge_engine.last_lines, edge_mask.shape, thickness=3)
                clean_mask = lines_to_mask(selected_lines, edge_mask.shape, thickness=4)
                if not np.any(clean_mask):
                    clean_mask = line_mask
                noise_mask = cv2.bitwise_and(edge_mask, cv2.bitwise_not(line_mask))

            drawing_started = time.perf_counter()

            with block("tracker.draw_bev"):
                bev_mask = self.lane_selector.draw_bev_view()

            with block("tracker.resize_input"):
                processing_frame = cv2.resize(
                    processing_input,
                    (self.edge_engine.input_width, self.edge_engine.input_height),
                    interpolation=cv2.INTER_AREA,
                )

            with block("tracker.draw_original"):
                original_view = self.lane_selector.draw_original_view(processing_frame)

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

            with block("tracker.draw_debug_panel"):
                debug_frame = draw_debug_panel(
                    clean_mask=clean_mask, bev_mask=bev_mask,
                    lane_state=lane_state,
                    raw_image=original_view,
                    camera_pitch_deg=self.ipm.pitch_deg,
                    physical_track_width_mm=self.physical_track_width_mm,
                    noise_mask=noise_mask,
                    semantic_mask=semantic_mask,
                )
            draw_ms = (time.perf_counter() - drawing_started) * 1000.0

            with block("tracker.capture_debug"):
                try:
                    self._capture_debug(
                        processing_input=processing_input,
                        lane_state=lane_state,
                        clean_mask=clean_mask,
                        bev_mask=bev_mask,
                        original_view=original_view,
                    )
                except Exception as error:
                    self.logger.warning(f"debug capture failed: {error}")

            process_time = (time.time() - start_time) * 1000
            self.last_timing = {
                "total_ms": process_time,
                "inference_ms": inference_ms,
                "model_inference_ms": self.edge_engine.last_inference_ms,
                "postprocess_ms": self.edge_engine.last_postprocess_ms,
                "fitting_ms": fitting_ms,
                "draw_ms": draw_ms,
            }
            if process_time > 10:
                self.logger.warning(f"巡线处理耗时: {process_time:.0f}ms")

            return offset_mm, is_intersection, debug_frame

        except Exception as e:
            self.logger.exception(f"巡线处理异常: {e}")
            return 0.0, False, frame.copy() if frame is not None else np.zeros((240, 320, 3), dtype=np.uint8)

        finally:
            self._write_timing_log()

    def _write_timing_log(self):
        """把本帧收集的阶段耗时追加写入日志文件（若启用）。"""
        if self.timing_log_path is None:
            return
        timings = get_frame_timings()
        if not timings:
            return
        now = time.strftime("%Y-%m-%d %H:%M:%S")
        total_sum = sum(ms for _, ms in timings)
        lines = [f"[{now}] frame={self._timing_frame_counter}"]
        for name, ms in timings:
            lines.append(f"  {name:<36s} {ms:9.3f} ms")
        lines.append(f"  {'TOTAL (sum of blocks)':<36s} {total_sum:9.3f} ms")
        lines.append("")
        try:
            with open(self.timing_log_path, "a", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
        except Exception as error:
            self.logger.warning(f"timing log write failed: {error}")

    def _render_debug_views(self, capture):
        """Render every available debug view from a capture dict."""
        from perception.diagnostics.lane_debug_viz import VIEW_BUILDERS
        views = {}
        for name, builder in VIEW_BUILDERS.items():
            try:
                img = builder(capture)
            except Exception as error:
                self.logger.warning(f"debug view '{name}' failed: {error}")
                img = None
            if img is not None:
                views[name] = img
        self.last_debug_views = views
        return views


    def _dump_debug_views(self):
        """Optionally write every debug view to disk for offline inspection."""
        if self.debug_dump_dir is None:
            return
        out_dir = Path(self.debug_dump_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        for name, img in self.last_debug_views.items():
            cv2.imwrite(
                str(out_dir / f"{self._debug_frame_idx:06d}_{name}.png"), img,
            )
        self._debug_frame_idx += 1

    def reset(self):
        """重置追踪器状态"""
        self.logger.info("LaneTracker 已重置")