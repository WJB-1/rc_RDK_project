#!/usr/bin/env python3
"""Run semantic lane inference offline and save diagnosis artifacts per image."""

import argparse
import csv
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


SCRIPT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_ROOT.parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def _json_value(value):
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return {"shape": list(value.shape), "dtype": str(value.dtype)}
    return value


def build_record(image_name, process_ms, mask_coverage, lane_state, capture, timing):
    gate = (capture or {}).get("semantic_lane") or {}
    stage_timings = (timing or {}).get("stages_ms") or {}
    record = {
        "image": image_name,
        "process_ms": round(float(process_ms), 3),
        "mask_coverage": round(float(mask_coverage), 6),
        "lane_method": (lane_state or {}).get("lane_method"),
        "frame_dropped": bool((lane_state or {}).get("frame_dropped", False)),
        "offset_mm": (lane_state or {}).get("pid_error_mm"),
        "yaw_deg": None,
        "gate_accepted": bool(gate.get("accepted", False)),
        "gate_reason": gate.get("fallback_reason"),
        "image_line_count": len(gate.get("image_lines") or []),
        "bev_line_count": len(gate.get("bev_lines") or []),
        "accepted_bev_count": len(gate.get("accepted_bev_lines") or []),
        "rejected_bev_count": len(gate.get("rejected_bev_lines") or []),
        "raw_hough_segment_count": int(gate.get("raw_hough_segment_count", 0)),
        "distance_gate_min_mm": gate.get("min_distance_mm"),
        "distance_gate_max_mm": gate.get("max_distance_mm"),
        "pair_count": len(gate.get("pair_measurements") or []),
    }
    measurements = gate.get("pair_measurements") or []
    if measurements:
        first_pair = measurements[0]
        record["first_pair_distance_mm"] = first_pair.get("normal_distance_mm")
        record["first_pair_angle_deg"] = first_pair.get("parallel_angle_deg")
    if (lane_state or {}).get("lane_angle_rad") is not None:
        record["yaw_deg"] = round(float(np.degrees(lane_state["lane_angle_rad"])), 4)
    for name, milliseconds in stage_timings.items():
        record[f"{name}_ms"] = round(float(milliseconds), 3)
    return record


def _load_settings(settings_path):
    settings = yaml.safe_load(settings_path.read_text(encoding="utf-8")) or {}
    try:
        from perception.profiles import apply_vision_profile
    except ImportError:
        return settings
    return apply_vision_profile(settings)


def _write_image(path, image):
    if image is None:
        return
    cv2.imwrite(str(path), image)


def _make_montage(images):
    resized = []
    for title, image in images:
        if image is None:
            image = np.zeros((384, 512, 3), dtype=np.uint8)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        height, width = image.shape[:2]
        scale = min(512 / width, 320 / height)
        panel = cv2.resize(image, (round(width * scale), round(height * scale)))
        canvas = np.zeros((360, 512, 3), dtype=np.uint8)
        y = 40 + (320 - panel.shape[0]) // 2
        x = (512 - panel.shape[1]) // 2
        canvas[y:y + panel.shape[0], x:x + panel.shape[1]] = panel
        cv2.putText(canvas, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (230, 230, 230), 2)
        resized.append(canvas)
    rows = [np.hstack(resized[index:index + 2]) for index in range(0, len(resized), 2)]
    if len(rows[-1].shape) == 3 and rows[-1].shape[1] == 512:
        rows[-1] = np.hstack((rows[-1], np.zeros_like(rows[-1])))
    return np.vstack(rows)


class SemanticRuntime:
    """Semantic-only runtime that deliberately avoids PiDiNet initialization."""

    def __init__(self, settings):
        from perception.algorithms.lane.builders import build_ipm, build_selector, build_semantic_engine
        from perception.algorithms.lane.config import load_lane_config
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector
        from perception.algorithms.lane.undistort import Undistorter
        from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer

        self.cfg = load_lane_config(settings)
        self.undistorter = Undistorter(self.cfg.undistort_camera)
        self.semantic_engine = build_semantic_engine(self.cfg)
        if self.semantic_engine is None:
            raise RuntimeError("semantic_lane is disabled or its model is unavailable")
        self.selector = build_selector(self.cfg, None, build_ipm(self.cfg), False)
        self.detector = SemanticLaneDetector(
            pixel_per_mm=self.cfg.pixel_per_mm,
            bev_width=self.cfg.canvas_w,
            lane_width_mm=self.cfg.lane_width_mm,
            distance_tolerance_mm=float(self.cfg.semantic_lane.get("distance_tolerance_mm", 20.0)),
            parallel_tolerance_deg=float(self.cfg.semantic_lane.get("parallel_tolerance_deg", 3.0)),
            min_segment_length_px=int(self.cfg.semantic_lane.get("min_segment_length_px", 30)),
            max_curve_residual_px=float(self.cfg.semantic_lane.get("max_curve_residual_px", 8.0)),
        )
        self.renderer = LaneSelectorRenderer(self.selector)

    @staticmethod
    def _overlay(image, mask):
        view = image.copy()
        pixels = np.asarray(mask) > 0
        tint = np.empty_like(view)
        tint[:] = (60, 220, 60)
        mixed = cv2.addWeighted(view, 0.55, tint, 0.45, 0)
        view[pixels] = mixed[pixels]
        return view

    def _bev(self, result):
        edge = np.asarray(result["edge_mask"], dtype=np.uint8)
        matrix = self.selector._matrix_for_profile("lane")
        warped = cv2.warpPerspective(edge, matrix, (self.cfg.canvas_w, self.cfg.canvas_h),
                                     flags=cv2.INTER_NEAREST)
        view = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        for segment in result["rejected_bev_lines"]:
            cv2.line(view, tuple(np.round(segment[0]).astype(int)), tuple(np.round(segment[1]).astype(int)),
                     (0, 90, 255), 2, cv2.LINE_AA)
        for segment in result["accepted_bev_lines"]:
            cv2.line(view, tuple(np.round(segment[0]).astype(int)), tuple(np.round(segment[1]).astype(int)),
                     (0, 255, 0), 3, cv2.LINE_AA)
        from perception.algorithms.lane.pipeline import LanePipeline
        LanePipeline._draw_semantic_gate_text(view, result)
        return view

    def _ground(self, result):
        view = self.renderer.render_new_ground_bev()
        projector = getattr(self.selector, "new_ground", None)
        if projector is None:
            return view
        height = view.shape[0]
        for line in result["image_lines"]:
            points = projector.source_line_to_ground(line)
            if len(points) < 2:
                continue
            pixels = [(round(40 + (x + 600.0) * 0.5), round(height - 40 - y * 0.5)) for x, y in points]
            cv2.polylines(view, [np.asarray(pixels, dtype=np.int32)], False, (0, 255, 0), 3, cv2.LINE_AA)
        return view

    def process(self, frame):
        stages = {}
        started = time.perf_counter()
        stage_started = time.perf_counter()
        semantic_width = self.semantic_engine.input_width
        semantic_height = self.semantic_engine.input_height
        semantic_image = cv2.resize(
            frame, (semantic_width, semantic_height), interpolation=cv2.INTER_AREA
        )
        stages["resize_ms"] = (time.perf_counter() - stage_started) * 1000
        stage_started = time.perf_counter()
        semantic_image = self.undistorter.apply(semantic_image)
        stages["undistort_ms"] = (time.perf_counter() - stage_started) * 1000
        stage_started = time.perf_counter()
        native_mask = self.semantic_engine.inference(semantic_image)
        stages["semantic_inference_ms"] = (time.perf_counter() - stage_started) * 1000
        image = cv2.resize(
            semantic_image, (self.cfg.input_width, self.cfg.input_height), interpolation=cv2.INTER_AREA
        )
        mask = cv2.resize(
            native_mask, (self.cfg.input_width, self.cfg.input_height), interpolation=cv2.INTER_NEAREST
        )
        stage_started = time.perf_counter()
        result = self.detector.analyze(mask, self.selector._matrix_for_profile("lane"))
        stages["boundary_gate_ms"] = (time.perf_counter() - stage_started) * 1000
        source_lines = result["accepted_image_lines"] if result["accepted"] else []
        lane_state = self.selector.analyze(source_lines, semantic_mask=mask)
        lane_state["lane_method"] = "semantic_boundary"
        lane_state["frame_dropped"] = not result["accepted"]
        stages["selector_ms"] = (time.perf_counter() - stage_started) * 1000
        stages["total_ms"] = (time.perf_counter() - started) * 1000
        return {
            "mask": mask, "lane_state": lane_state, "semantic_gate": result,
            "timing": {"total_ms": stages["total_ms"], "stages_ms": stages},
            "views": {"semantic_overlay": self._overlay(semantic_image, native_mask), "semantic_bev": self._bev(result),
                      "semantic_ground": self._ground(result)},
        }


def diagnose_image(runtime, image_path, output_dir):
    frame = cv2.imread(str(image_path))
    if frame is None:
        raise RuntimeError("image decode failed")
    started = time.perf_counter()
    result = runtime.process(frame)
    process_ms = (time.perf_counter() - started) * 1000.0

    lane_state = result["lane_state"]
    capture = {"semantic_lane": result["semantic_gate"]}
    views = result["views"]
    semantic_mask = result["mask"]
    mask_coverage = 0.0 if semantic_mask is None else float(np.count_nonzero(semantic_mask)) / semantic_mask.size
    record = build_record(
        image_path.name, process_ms, mask_coverage, lane_state, capture, result["timing"]
    )

    stem_dir = output_dir / image_path.stem
    stem_dir.mkdir(parents=True, exist_ok=True)
    _write_image(stem_dir / "01_original.jpg", frame)
    _write_image(stem_dir / "02_mask.png", semantic_mask)
    _write_image(stem_dir / "03_overlay.jpg", views.get("semantic_overlay"))
    _write_image(stem_dir / "04_parallel_bev.jpg", views.get("semantic_bev"))
    _write_image(stem_dir / "05_ground.jpg", views.get("semantic_ground"))
    montage = _make_montage([
        ("original", frame), ("semantic mask", semantic_mask),
        ("mask overlay", views.get("semantic_overlay")),
        ("parallel BEV + gate", views.get("semantic_bev")),
        ("ground projection", views.get("semantic_ground")),
    ])
    _write_image(stem_dir / "00_montage.jpg", montage)
    detail = {"record": record, "lane_state": _json_value(lane_state),
              "semantic_gate": _json_value(capture.get("semantic_lane") or {}),
              "timing": _json_value(result["timing"])}
    (stem_dir / "diagnostics.json").write_text(json.dumps(detail, indent=2, ensure_ascii=False), encoding="utf-8")
    return record


def main():
    parser = argparse.ArgumentParser(description="Offline semantic-lane inference diagnosis")
    parser.add_argument("--img-dir", type=Path, default=SCRIPT_ROOT / "img")
    parser.add_argument("--settings", type=Path, default=PROJECT_ROOT / "config" / "settings.yaml")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--warmup", type=int, default=2)
    args = parser.parse_args()

    images = sorted(path for path in args.img_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
    if not images:
        raise SystemExit(f"no images found: {args.img_dir}")
    output_dir = args.output_dir or SCRIPT_ROOT / "semantic_diagnose_output" / datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir.mkdir(parents=True, exist_ok=True)

    settings = _load_settings(args.settings)
    settings.setdefault("debug", {})["timing_enabled"] = False
    runtime = SemanticRuntime(settings)

    warmup = cv2.imread(str(images[0]))
    for _ in range(max(args.warmup, 0)):
        runtime.process(warmup)

    records = []
    for index, image_path in enumerate(images, start=1):
        try:
            record = diagnose_image(runtime, image_path, output_dir)
            records.append(record)
            print(f"[{index}/{len(images)}] {image_path.name}: gate={record['gate_accepted']} "
                  f"reason={record['gate_reason']} total={record['process_ms']:.1f} ms")
        except Exception as error:
            print(f"[{index}/{len(images)}] {image_path.name}: ERROR {error}")

    if records:
        fields = sorted({key for record in records for key in record})
        with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8-sig") as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)
    (output_dir / "summary.json").write_text(json.dumps(records, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"output: {output_dir}")


if __name__ == "__main__":
    main()
