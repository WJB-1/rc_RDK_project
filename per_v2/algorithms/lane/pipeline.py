# perception/algorithms/lane/pipeline.py
"""车道检测单帧流水线：去畸变 → 边缘推理 → 语义门控 → selector → renderer → debug 面板。

可视化全部委托给 LaneSelectorRenderer，selector 只负责算法。
"""
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from perception.algorithms.timing import reset_frame, block, get_frame_timings
from perception.algorithms.mask_utils import clean_mask_by_cc
from perception.algorithms.pidinet_lane import lines_to_mask
from perception.diagnostics.ipm_drawer import draw_debug_panel

from .types import LanePipelineResult


def _make_renderer(selector):
    """延迟 import，避免循环依赖。"""
    from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer
    return LaneSelectorRenderer(selector)


class LanePipeline:
    def __init__(self, cfg, undistorter, edge_engine, semantic_engine,
                 selector, ipm, semantic_gate_enabled, settings):
        self.cfg = cfg
        self.undistorter = undistorter
        self.edge_engine = edge_engine
        self.semantic_engine = semantic_engine
        self.selector = selector
        self.ipm = ipm
        self.semantic_gate_enabled = semantic_gate_enabled
        self.settings = settings
        self.debug = settings.get("debug", {}).get("show_video", True)

        # debug capture 需要的上下文
        self.bev_cfg = cfg.bev_cfg
        self.matching_cfg = cfg.matching_cfg
        self.template_x_at_ref_mm = cfg.template_x_at_ref_mm

        # 阶段耗时日志
        debug_cfg = settings.get("debug", {}) or {}
        self.timing_enabled = bool(debug_cfg.get("timing_enabled", True))
        self.timing_log_path = None
        if self.timing_enabled:
            log_path = debug_cfg.get("timing_log_path", "logs/lane_timing.log")
            if not Path(log_path).is_absolute():
                log_path = str(Path(__file__).parents[3] / log_path)
            self.timing_log_path = log_path
            try:
                Path(self.timing_log_path).parent.mkdir(parents=True, exist_ok=True)
            except Exception:
                self.timing_log_path = None
        self._timing_frame_counter = 0

        self.last_debug_capture = {}

    # ------------------------------------------------------------------
    # 单帧处理
    # ------------------------------------------------------------------
    def process(self, frame: np.ndarray) -> LanePipelineResult:
        start_time = time.time()
        reset_frame()
        self._timing_frame_counter += 1

        try:
            with block("tracker.undistort"):
                processing_input = self.undistorter.apply(frame)

            with block("tracker.edge_inference"):
                edge_mask = self.edge_engine.inference(processing_input)

            semantic_mask = None
            if self.semantic_gate_enabled and self.semantic_engine is not None:
                with block("tracker.semantic_inference"):
                    semantic_raw = self.semantic_engine.inference(processing_input)
                    semantic_mask, _ = clean_mask_by_cc(
                        semantic_raw, min_bottom_y=semantic_raw.shape[0] - 10,
                    )

            with block("tracker.selector_analyze"):
                lane_state = self.selector.analyze(
                    self.edge_engine.last_lines, semantic_mask=semantic_mask,
                )

            selected_lines = self.selector.detected_source_lines
            with block("tracker.lines_to_mask"):
                line_mask = lines_to_mask(self.edge_engine.last_lines, edge_mask.shape, thickness=3)
                clean_mask = lines_to_mask(selected_lines, edge_mask.shape, thickness=4)
                if not np.any(clean_mask):
                    clean_mask = line_mask
                noise_mask = cv2.bitwise_and(edge_mask, cv2.bitwise_not(line_mask))

            renderer = _make_renderer(self.selector)

            with block("tracker.draw_bev"):
                bev_mask = renderer.draw_bev_view()

            with block("tracker.resize_input"):
                processing_frame = cv2.resize(
                    processing_input,
                    (self.edge_engine.input_width, self.edge_engine.input_height),
                    interpolation=cv2.INTER_AREA,
                )

            with block("tracker.draw_original"):
                original_view = renderer.draw_original_view(processing_frame)

            lane_state["raw_line_count"] = len(self.edge_engine.last_raw_lines)
            lane_state["line_count"] = len(self.edge_engine.last_lines)
            lane_state["line_method"] = (
                "HoughLinesP + PCA merge + six-line template distance"
                if hasattr(self.selector, "template_x_at_ref_mm")
                else "HoughLinesP + PCA merge (a3d8)"
            )

            with block("tracker.draw_debug_panel"):
                debug_frame = draw_debug_panel(
                    clean_mask=clean_mask,
                    bev_mask=bev_mask,
                    lane_state=lane_state,
                    raw_image=original_view,
                    camera_pitch_deg=self.ipm.pitch_deg,
                    physical_track_width_mm=self.cfg.lane_width_mm,
                    noise_mask=noise_mask,
                    semantic_mask=semantic_mask,
                )

            with block("tracker.capture_debug"):
                try:
                    self._capture_debug(
                        processing_input=processing_input,
                        lane_state=lane_state,
                        clean_mask=clean_mask,
                        bev_mask=bev_mask,
                        original_view=original_view,
                        renderer=renderer,
                    )
                except Exception as error:
                    self.last_debug_capture = {}
                    print(f"[LanePipeline] debug capture failed: {error}")

            timing = {
                "total_ms": (time.time() - start_time) * 1000,
                "inference_ms": self.edge_engine.last_inference_ms,
                "postprocess_ms": self.edge_engine.last_postprocess_ms,
            }

            return LanePipelineResult(
                offset_mm=lane_state["pid_error_mm"],
                is_intersection=lane_state["crossroad_detected"],
                debug_frame=debug_frame,
                lane_state=lane_state,
                bev_mask=bev_mask,
                original_view=original_view,
                seg_mask=semantic_mask if semantic_mask is not None else clean_mask,
                timing=timing,
                debug_views={},
                debug_capture=dict(self.last_debug_capture),
            )
        finally:
            self._write_timing_log()

    # ------------------------------------------------------------------
    # Debug capture
    # ------------------------------------------------------------------
    def _capture_debug(self, processing_input, lane_state, clean_mask,
                       bev_mask, original_view, renderer):
        edge_engine = self.edge_engine
        lane_selector = self.selector

        capture = {
            "enhanced": getattr(edge_engine, "last_enhanced", None),
            "edge_prob": getattr(edge_engine, "last_probability", None),
            "binary": getattr(edge_engine, "last_binary", None),
            "closed": getattr(edge_engine, "last_closed", None),
            "labels": getattr(edge_engine, "last_label_map", None),
            "n_labels": int(getattr(edge_engine, "last_n_labels", 0)),
            "skeleton": getattr(edge_engine, "last_skeleton", None),
            "hough_lines": list(getattr(edge_engine, "last_raw_lines", []) or []),

            "clean_mask": clean_mask,
            "bev_mask": bev_mask,
            "original_view": original_view,

            "ground_segments": list(getattr(lane_selector, "ground_segments", []) or []),
            "merged": list(getattr(lane_selector, "merged_segments", []) or []),
            "candidates": dict(getattr(lane_selector, "candidate_slots", {}) or {}),
            "template": dict(self.template_x_at_ref_mm or {}),
            "candidate_radius_mm": float(
                self.matching_cfg.get("candidate_radius_first_frame_mm", 40.0)
            ),
            "y_ref_mm": float(self.matching_cfg.get("y_ref_mm", 500.0)),

            "result": getattr(lane_selector, "last_match_result", None),
            "lane_state": dict(lane_state or {}),

            "original_input": processing_input,
            "ground_to_pixel": getattr(self.ipm, "ground_to_pixel", None),
            "scale_up_x": 1.0,
            "scale_up_y": 1.0,

            "canvas": getattr(lane_selector, "bev_canvas", None),
        }

        if capture["canvas"] is None:
            try:
                from perception.diagnostics.lane_debug_viz import BevCanvas
                capture["canvas"] = BevCanvas(
                    x_min=self.bev_cfg.get("x_min", -600.0),
                    x_max=self.bev_cfg.get("x_max", 600.0),
                    y_min=self.bev_cfg.get("y_min", -250.0),
                    y_max=self.bev_cfg.get("y_max", 800.0),
                    ppm=self.bev_cfg.get("ppm", 0.9),
                )
            except Exception:
                capture["canvas"] = None

        # 新地面系可视化：只有 Template selector 有 new_ground 属性
        if getattr(lane_selector, "new_ground", None) is not None:
            try:
                capture["new_ground_bev"] = renderer.render_new_ground_bev()
            except Exception as error:
                print(f"[LanePipeline] render_new_ground_bev failed: {error}")
                capture["new_ground_bev"] = None

            try:
                capture["new_ground_original"] = (
                    renderer.render_original_with_ground_centerline(processing_input)
                )
            except Exception as error:
                print(
                    f"[LanePipeline] render_original_with_ground_centerline failed: {error}"
                )
                capture["new_ground_original"] = None
        else:
            capture["new_ground_bev"] = None
            capture["new_ground_original"] = None

        self.last_debug_capture = capture
        return capture

    # ------------------------------------------------------------------
    # 阶段耗时日志
    # ------------------------------------------------------------------
    def _write_timing_log(self):
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
            print(f"[LanePipeline] timing log write failed: {error}")

    def reset(self):
        self.last_debug_capture = {}