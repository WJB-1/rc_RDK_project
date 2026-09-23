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
        semantic_cfg = getattr(cfg, "semantic_lane", {}) or {}
        configured_mode = str(semantic_cfg.get("mode", "auto")).lower()
        self.semantic_lane_mode = (
            configured_mode if semantic_lane_detector is not None else "template"
        )
        if self.semantic_lane_mode not in {"template", "semantic", "auto"}:
            self.semantic_lane_mode = "auto" if semantic_lane_detector is not None else "template"
        self._auto_template_next = False
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

            run_template = self._should_run_template()
            edge_mask = np.zeros(processing_input.shape[:2], dtype=np.uint8)
            if run_template:
                with block("tracker.edge_inference"):
                    edge_mask = self.edge_engine.inference(processing_input)

            semantic_mask = None
            semantic_result = None
            semantic_requested = not run_template and self.semantic_lane_mode in {"semantic", "auto"}
            if semantic_requested and self.semantic_engine is not None:
                with block("tracker.semantic_inference"):
                    semantic_raw = self.semantic_engine.inference(processing_input)
                    semantic_mask, _ = clean_mask_by_cc(
                        semantic_raw, min_bottom_y=semantic_raw.shape[0] - 10,
                    )

            with block("tracker.selector_analyze"):
                if self.semantic_lane_detector is not None and semantic_mask is not None:
                    semantic_result = self.semantic_lane_detector.analyze(
                        semantic_mask, self.selector._matrix_for_profile("lane")
                    )
                semantic_accepted = bool(semantic_result and semantic_result["accepted"])
                if run_template:
                    source_lines = self.edge_engine.last_lines
                    lane_method = "template"
                elif semantic_accepted:
                    source_lines = semantic_result["accepted_image_lines"]
                    lane_method = "semantic_boundary"
                elif self.semantic_lane_mode == "auto":
                    source_lines = self.edge_engine.last_lines
                    lane_method = "template_fallback"
                else:
                    source_lines = []
                    lane_method = "semantic_invalid_drop"
                lane_state = self.selector.analyze(
                    source_lines,
                    semantic_mask=semantic_mask,
                )
                lane_state["lane_method"] = lane_method
                lane_state["frame_dropped"] = lane_method == "semantic_invalid_drop"
                if self.semantic_lane_mode == "auto":
                    self._auto_template_next = lane_method == "semantic_invalid_drop"

            selected_lines = self.selector.detected_source_lines
            with block("tracker.lines_to_mask"):
                line_mask = lines_to_mask(
                    self.edge_engine.last_lines if run_template else source_lines,
                    edge_mask.shape,
                    thickness=3,
                )
                clean_mask = lines_to_mask(selected_lines, edge_mask.shape, thickness=4)
                if not np.any(clean_mask):
                    clean_mask = line_mask
                noise_mask = cv2.bitwise_and(edge_mask, cv2.bitwise_not(line_mask))

            renderer = _make_renderer(self.selector)

            if run_template:
                with block("tracker.draw_bev"):
                    bev_mask = renderer.draw_bev_view()
                with block("tracker.draw_original"):
                    original_view = renderer.draw_original_view(processing_input)
            else:
                with block("tracker.draw_bev"):
                    bev_mask = renderer.render_new_ground_bev()
                original_view = processing_input.copy()

            lane_state["raw_line_count"] = len(self.edge_engine.last_raw_lines) if run_template else 0
            lane_state["line_count"] = len(self.edge_engine.last_lines) if run_template else len(source_lines)
            lane_state["line_method"] = (
                "semantic boundary + next-frame template fallback"
                if self.semantic_lane_mode == "auto" else "semantic boundary (invalid frames dropped)"
                if self.semantic_lane_mode == "semantic" else "HoughLinesP + PCA merge + six-line template distance"
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
                            ground_bev=bev_mask if not run_template else None,
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
                       bev_mask, original_view, renderer, semantic_mask=None, semantic_result=None,
                       ground_bev=None):
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
                "ground_bev": ground_bev if ground_bev is not None else renderer.render_new_ground_bev(),
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
                "template_match_diagnostics": dict(
                    getattr(lane_selector, "_last_template_match_diagnostics", {}) or {}
                ),
            },
        }
        if semantic_mask is not None:
            semantic_debug = semantic_result or {
                "edge_mask": cv2.Canny(np.asarray(semantic_mask, dtype=np.uint8), 50, 150),
                "accepted_bev_lines": [],
                "rejected_bev_lines": [],
            }
            capture["semantic_lane"] = semantic_result or {
                "accepted": False,
                "fallback_reason": "semantic_detector_no_result",
                "image_lines": [],
                "accepted_image_lines": [],
                "accepted_bev_lines": [],
                "rejected_bev_lines": [],
            }
            capture["lane_views"].update({
                "semantic_overlay": self._draw_semantic_overlay(processing_input, semantic_mask),
                "semantic_bev": self._draw_semantic_bev(
                    semantic_debug, self.selector._matrix_for_profile("lane")
                ),
                "semantic_ground": self._draw_semantic_ground(renderer, semantic_result),
            })

        self.last_debug_capture = capture
        return capture

    @staticmethod
    def _draw_semantic_overlay(image, mask):
        view = image.copy()
        if mask is not None:
            mask_pixels = np.asarray(mask) > 0
            overlay = np.empty_like(view)
            overlay[:] = (60, 220, 60)
            blended = cv2.addWeighted(view, 0.55, overlay, 0.45, 0)
            view[mask_pixels] = blended[mask_pixels]
        return view

    def _draw_semantic_ground(self, renderer, semantic_result):
        view = renderer.render_new_ground_bev()
        projector = getattr(self.selector, "new_ground", None)
        if projector is None or not semantic_result:
            return view
        height, width = view.shape[:2]
        def ground_pixel(point):
            x, y = point
            return (int(round(40 + (x + 600.0) * 0.5)),
                    int(round(height - 40 - y * 0.5)))
        for line in semantic_result.get("image_lines", []):
            points = projector.source_line_to_ground(line)
            if len(points) < 2:
                continue
            pixels = [ground_pixel(point) for point in points]
            cv2.polylines(view, [np.asarray(pixels, dtype=np.int32)], False, (0, 255, 0), 3)
        return view

    def _draw_semantic_bev(self, result, parallel_matrix):
        edge_mask = np.asarray(result.get("edge_mask"), dtype=np.uint8)
        if edge_mask.ndim != 2:
            edge_mask = np.zeros((self.selector.canvas_h, self.selector.canvas_w), dtype=np.uint8)
        warped_edges = cv2.warpPerspective(
            edge_mask,
            np.asarray(parallel_matrix, dtype=np.float64),
            (self.selector.canvas_w, self.selector.canvas_h),
            flags=cv2.INTER_NEAREST,
        )
        view = cv2.cvtColor(warped_edges, cv2.COLOR_GRAY2BGR)
        for segment in result.get("rejected_bev_lines", []):
            cv2.line(view, tuple(np.round(segment[0]).astype(int)), tuple(np.round(segment[1]).astype(int)), (0, 90, 255), 2)
        for segment in result.get("accepted_bev_lines", []):
            cv2.line(view, tuple(np.round(segment[0]).astype(int)), tuple(np.round(segment[1]).astype(int)), (0, 255, 0), 3)
        self._draw_semantic_gate_text(view, result)
        return view

    @staticmethod
    def _draw_semantic_gate_text(view, result):
        """Render semantic-pair gate measurements without hiding BEV evidence."""
        accepted = bool(result.get("accepted", False))
        min_distance = result.get("min_distance_mm")
        max_distance = result.get("max_distance_mm")
        raw_count = int(result.get("raw_hough_segment_count", 0))
        fitted_count = len(result.get("bev_lines") or [])
        status = "PASS" if accepted else "REJECT"
        status_colour = (0, 220, 0) if accepted else (0, 90, 255)
        lines = [
            (f"semantic pair gate: {status}", status_colour),
            (f"Hough segments: {raw_count}; fitted boundaries: {fitted_count}", (230, 230, 230)),
        ]
        if min_distance is not None and max_distance is not None:
            lines.append((f"normal-distance gate: {float(min_distance):.1f}..{float(max_distance):.1f} mm", (230, 230, 230)))
        measurements = result.get("pair_measurements") or []
        if not measurements:
            lines.append(("normal distance: unavailable (need 2 fitted boundaries)", (0, 210, 255)))
        else:
            for measurement in measurements[:4]:
                distance = measurement.get("normal_distance_mm")
                angle = measurement.get("parallel_angle_deg")
                distance_text = "n/a" if distance is None else f"{float(distance):.1f} mm"
                angle_text = "n/a" if angle is None else f"{float(angle):.1f} deg"
                failed = []
                if not measurement.get("y_overlap_ok"):
                    failed.append("no-overlap")
                if not measurement.get("parallel_ok"):
                    failed.append("non-parallel")
                if not measurement.get("distance_ok"):
                    failed.append("distance")
                verdict = "PASS" if not failed else ", ".join(failed)
                colour = (0, 220, 0) if not failed else (0, 210, 255)
                lines.append((
                    f"pair {measurement.get('i')}-{measurement.get('j')}: d={distance_text}, angle={angle_text} [{verdict}]",
                    colour,
                ))
        if not accepted and result.get("fallback_reason"):
            lines.append((f"reason: {result['fallback_reason']}", (0, 140, 255)))

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale, thickness, line_height = 0.43, 1, 17
        panel_height = min(view.shape[0], 10 + line_height * len(lines))
        panel_width = min(view.shape[1], 500)
        shade = view.copy()
        cv2.rectangle(shade, (0, 0), (panel_width, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(shade, 0.72, view, 0.28, 0, dst=view)
        for index, (text, colour) in enumerate(lines):
            cv2.putText(view, text, (7, 16 + index * line_height), font, scale, colour, thickness, cv2.LINE_AA)

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
        self._auto_template_next = False

    def _should_run_template(self) -> bool:
        return self.semantic_lane_mode == "template" or (
            self.semantic_lane_mode == "auto" and self._auto_template_next
        )

    def set_semantic_lane_mode(self, mode: str) -> str:
        mode = str(mode).strip().lower()
        if mode not in {"template", "semantic", "auto"}:
            raise ValueError("lane detection mode must be template, semantic, or auto")
        if mode != "template" and (
            self.semantic_lane_detector is None or self.semantic_engine is None
        ):
            raise RuntimeError("semantic lane detector is unavailable")
        self.semantic_lane_mode = mode
        self._auto_template_next = False
        return mode

    def set_debug_capture_enabled(self, enabled: bool) -> bool:
        self.debug_capture_enabled = bool(enabled)
        if not self.debug_capture_enabled:
            self.last_debug_capture = {}
        return self.debug_capture_enabled

    def set_debug_render_enabled(self, enabled: bool) -> bool:
        self.debug_render_enabled = bool(enabled)
        return self.debug_render_enabled
