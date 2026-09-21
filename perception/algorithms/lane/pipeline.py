# perception/algorithms/lane/pipeline.py
"""车道检测单帧流水线：去畸变 → 边缘推理 → 语义门控 → selector → renderer → debug 面板。

可视化全部委托给 LaneSelectorRenderer，selector 只负责算法。
"""
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from perception.algorithms.core.timing import reset_frame, block, get_frame_timings
from perception.algorithms.core.mask_utils import clean_mask_by_cc
from .line_detection import lines_to_mask

from .types import LanePipelineResult


def _make_renderer(selector):
    """延迟 import，避免循环依赖。"""
    from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer
    return LaneSelectorRenderer(selector)


class LanePipeline:
    def __init__(self, cfg, undistorter, edge_engine, semantic_engine,
                 selector, ipm, semantic_gate_enabled, settings, semantic_lane_detector=None):
        self.cfg = cfg
        self.undistorter = undistorter
        self.edge_engine = edge_engine
        self.semantic_engine = semantic_engine
        self.selector = selector
        self.ipm = ipm
        self.semantic_gate_enabled = semantic_gate_enabled
        self.semantic_lane_detector = semantic_lane_detector
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
        self.debug_render_enabled = bool(debug_cfg.get("render_enabled", False))
        self.debug_capture_enabled = bool(debug_cfg.get("capture_enabled", False))

    # ------------------------------------------------------------------
    # 单帧处理
    # ------------------------------------------------------------------
    def process(self, frame: np.ndarray) -> LanePipelineResult:
        start_time = time.time()
        reset_frame()
        self._timing_frame_counter += 1

        try:
            with block("tracker.resize_input"):
                processing_input = cv2.resize(
                    frame,
                    (self.edge_engine.input_width, self.edge_engine.input_height),
                    interpolation=cv2.INTER_AREA,
                )

            with block("tracker.undistort"):
                processing_input = self.undistorter.apply(processing_input)

            with block("tracker.edge_inference"):
                edge_mask = self.edge_engine.inference(processing_input)

            semantic_mask = None
            if (self.semantic_gate_enabled or self.semantic_lane_detector is not None) and self.semantic_engine is not None:
                with block("tracker.semantic_inference"):
                    semantic_raw = self.semantic_engine.inference(processing_input)
                    semantic_mask, _ = clean_mask_by_cc(
                        semantic_raw, min_bottom_y=semantic_raw.shape[0] - 10,
                    )

            semantic_result = None
            with block("tracker.selector_analyze"):
                if self.semantic_lane_detector is not None and semantic_mask is not None:
                    semantic_result = self.semantic_lane_detector.analyze(
                        semantic_mask, self.selector._matrix_for_profile("lane")
                    )
                semantic_lines = semantic_result.get("accepted_image_lines", []) if semantic_result else []
                lane_state = self.selector.analyze(
                    semantic_lines if semantic_result and semantic_result["accepted"] else self.edge_engine.last_lines,
                    semantic_mask=semantic_mask,
                )
                lane_state["lane_method"] = (
                    "semantic_boundary" if semantic_result and semantic_result["accepted"]
                    else "template_fallback"
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

            with block("tracker.draw_original"):
                original_view = renderer.draw_original_view(processing_input)

            lane_state["raw_line_count"] = len(self.edge_engine.last_raw_lines)
            lane_state["line_count"] = len(self.edge_engine.last_lines)
            lane_state["line_method"] = (
                "semantic boundary + template fallback"
                if self.semantic_lane_detector is not None else "HoughLinesP + PCA merge + six-line template distance"
                if hasattr(self.selector, "template_x_at_ref_mm")
                else "HoughLinesP + PCA merge (a3d8)"
            )

            debug_frame = original_view.copy()

            if self.debug_capture_enabled:
                with block("tracker.capture_debug"):
                    try:
                        self._capture_debug(
                            raw_frame=frame,
                            processing_input=processing_input,
                            lane_state=lane_state,
                            clean_mask=clean_mask,
                            bev_mask=bev_mask,
                            original_view=original_view,
                            renderer=renderer,
                            semantic_mask=semantic_mask,
                            semantic_result=semantic_result,
                        )
                    except Exception as error:
                        self.last_debug_capture = {}
                        print(f"[LanePipeline] debug capture failed: {error}")

            timing = {
                "total_ms": (time.time() - start_time) * 1000,
                "inference_ms": self.edge_engine.last_inference_ms,
                "postprocess_ms": self.edge_engine.last_postprocess_ms,
                "stages_ms": dict(get_frame_timings()),
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
    def _capture_debug(self, raw_frame, processing_input, lane_state, clean_mask,
                       bev_mask, original_view, renderer, semantic_mask=None, semantic_result=None):
        edge_engine = self.edge_engine
        lane_selector = self.selector
        hough_view = raw_frame.copy()
        scale_x = hough_view.shape[1] / float(processing_input.shape[1])
        scale_y = hough_view.shape[0] / float(processing_input.shape[0])
        for line in getattr(edge_engine, "last_lines", []) or []:
            cv2.line(
                hough_view,
                (round(line["x1"] * scale_x), round(line["y1"] * scale_y)),
                (round(line["x2"] * scale_x), round(line["y2"] * scale_y)),
                (0, 255, 0), 2, cv2.LINE_AA,
            )

        capture = {
            "lane_views": {
                "binary": getattr(edge_engine, "last_binary", None),
                "hough": hough_view,
                "lane_bev": bev_mask,
                "ground_bev": renderer.render_new_ground_bev(),
                "overlay": original_view,
            },
            "metrics": {
                "raw_hough_count": len(getattr(edge_engine, "last_raw_lines", []) or []),
                "merged_line_count": len(getattr(edge_engine, "last_lines", []) or []),
                "template_confidence": lane_state.get("template_confidence"),
                "template_residual_mm": lane_state.get("template_residual_mm"),
                "template_delta_mm": lane_state.get("template_delta_mm"),
                "lane_angle_deg": (getattr(lane_selector, "final_pair", None) or {}).get(
                    "lane_angle_deg"
                ),
                "offset_mm": lane_state.get("pid_error_mm"),
                "theta_source": lane_state.get("lane_angle_source"),
            },
        }
        if semantic_result is not None:
            capture["semantic_lane"] = semantic_result
            capture["lane_views"].update({
                "semantic_overlay": self._draw_semantic_overlay(processing_input, semantic_mask),
                "semantic_bev": self._draw_semantic_bev(semantic_result),
                "semantic_ground": renderer.render_new_ground_bev(),
            })

        self.last_debug_capture = capture
        return capture

    @staticmethod
    def _draw_semantic_overlay(image, mask):
        view = image.copy()
        if mask is not None:
            overlay = np.zeros_like(view)
            overlay[np.asarray(mask) > 0] = (60, 220, 60)
            view = cv2.addWeighted(view, 0.72, overlay, 0.45, 0)
        return view

    def _draw_semantic_bev(self, result):
        view = np.zeros((self.selector.canvas_h, self.selector.canvas_w, 3), dtype=np.uint8)
        for segment in result.get("rejected_bev_lines", []):
            cv2.line(view, tuple(np.round(segment[0]).astype(int)), tuple(np.round(segment[1]).astype(int)), (0, 90, 255), 2)
        for segment in result.get("accepted_bev_lines", []):
            cv2.line(view, tuple(np.round(segment[0]).astype(int)), tuple(np.round(segment[1]).astype(int)), (0, 255, 0), 3)
        return view

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

    def set_debug_capture_enabled(self, enabled: bool) -> bool:
        self.debug_capture_enabled = bool(enabled)
        if not self.debug_capture_enabled:
            self.last_debug_capture = {}
        return self.debug_capture_enabled

    def set_debug_render_enabled(self, enabled: bool) -> bool:
        self.debug_render_enabled = bool(enabled)
        return self.debug_render_enabled
