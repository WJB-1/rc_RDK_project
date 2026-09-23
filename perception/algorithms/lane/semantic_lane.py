"""Binary semantic-mask boundary extraction and BEV distance gating."""

import math

import cv2
import numpy as np

from .angle_template import template_positions_for_angle
from .line_detection import detect_lines, merge_lines
from .line_geometry import (
    _line_normal_distance, _segment_parallel_angle_deg, _y_overlap_px,
)


class SemanticLaneDetector:
    """Extract complete left/right mask boundaries before accepting a BEV lane pair."""

    def __init__(self, pixel_per_mm, bev_width, lane_width_mm,
                 distance_tolerance_mm=20.0, parallel_tolerance_deg=3.0,
                 min_segment_length_px=30, max_curve_residual_px=8.0,
                 template_pair_ids=("0", "1"), template_x_at_ref_mm=None,
                 component_gap_px=10, distance_tolerance_ratio=0.5):
        self.pixel_per_mm = float(pixel_per_mm)
        self.bev_width = int(bev_width)
        self.lane_width_mm = float(lane_width_mm)
        self.distance_tolerance_mm = float(distance_tolerance_mm)
        self.parallel_tolerance_deg = float(parallel_tolerance_deg)
        self.min_segment_length_px = int(min_segment_length_px)
        self.max_curve_residual_px = float(max_curve_residual_px)
        self.template_pair_ids = tuple(template_pair_ids)
        self.template_x_at_ref_mm = dict(template_x_at_ref_mm or {})
        self.component_gap_px = max(0, int(component_gap_px))
        self.distance_tolerance_ratio = max(0.0, float(distance_tolerance_ratio))

    def _select_ground_component(self, mask, bottom_margin_px=10):
        binary = np.where(np.asarray(mask) > 0, 255, 0).astype(np.uint8)
        connected = binary
        if self.component_gap_px:
            kernel = np.ones((self.component_gap_px + 1, 1), dtype=np.uint8)
            connected = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(connected, 8)
        height = binary.shape[0]
        candidates = []
        for label in range(1, count):
            x, y, width, component_height, area = stats[label]
            touches_ground = y + component_height >= height - int(bottom_margin_px)
            if touches_ground:
                candidates.append((int(area), label))
        if not candidates:
            return binary, {"label": None, "area_px": int(np.count_nonzero(binary)),
                            "bbox": None, "fallback": "no_ground_touching_component"}
        area, label = max(candidates)
        selected = np.where(labels == label, 255, 0).astype(np.uint8)
        return selected, {"label": label, "area_px": area, "bbox": [int(v) for v in stats[label, :4]]}

    @staticmethod
    def _line_from_points(points):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        if len(points) < 2:
            return None
        vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).reshape(4)
        direction = np.array([vx, vy], dtype=np.float64)
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6 or abs(direction[1]) < 1e-6:
            return None
        direction /= norm
        origin = np.array([x0, y0], dtype=np.float64)
        projections = (points - origin) @ direction
        first = origin + direction * projections.min()
        second = origin + direction * projections.max()
        normal = np.array([-direction[1], direction[0]])
        residual = float(np.mean(np.abs((points - origin) @ normal)))
        return {
            "x1": float(first[0]), "y1": float(first[1]),
            "x2": float(second[0]), "y2": float(second[1]),
            "length": float(np.linalg.norm(second - first)),
            "curve_residual_px": residual,
        }

    @staticmethod
    def _project_line(line, matrix):
        points = np.array([[[line["x1"], line["y1"]], [line["x2"], line["y2"]]]], dtype=np.float32)
        return cv2.perspectiveTransform(points, np.asarray(matrix, dtype=np.float64))[0]

    @staticmethod
    def _project_points(points, matrix):
        values = np.asarray(points, dtype=np.float32).reshape(1, -1, 2)
        if values.shape[1] == 0:
            return np.empty((0, 2), dtype=np.float32)
        return cv2.perspectiveTransform(values, np.asarray(matrix, dtype=np.float64))[0]

    def _extract_side_lines(self, mask):
        binary = np.asarray(mask, dtype=np.uint8) > 0
        edge = np.zeros_like(np.asarray(mask, dtype=np.uint8))
        sides = {"left": [], "right": []}
        height = binary.shape[0]
        for y in range(height):
            xs = np.flatnonzero(binary[y])
            if not len(xs):
                continue
            left_x, right_x = int(xs[0]), int(xs[-1])
            edge[y, left_x] = 255
            edge[y, right_x] = 255
            sides["left"].append((left_x, y))
            if right_x != left_x:
                sides["right"].append((right_x, y))
        raw_lines = detect_lines(edge)
        merged_lines = merge_lines(raw_lines, min_length=self.min_segment_length_px)
        lines = []
        boundary_points = {}
        side_arrays = {
            side: np.asarray(points, dtype=np.float32) for side, points in sides.items() if points
        }
        for line_index, line in enumerate(merged_lines):
            midpoint_y = (line["y1"] + line["y2"]) * 0.5
            midpoint_x = (line["x1"] + line["x2"]) * 0.5
            distances = {}
            for side, points in side_arrays.items():
                expected_x = np.interp(midpoint_y, points[:, 1], points[:, 0])
                distances[side] = abs(midpoint_x - expected_x)
            if not distances:
                continue
            side = min(distances, key=distances.get)
            points = side_arrays[side]
            low_y, high_y = sorted((line["y1"], line["y2"]))
            segment = points[(points[:, 1] >= low_y) & (points[:, 1] <= high_y)]
            if len(segment) < 2:
                segment = np.asarray([[line["x1"], line["y1"]],
                                      [line["x2"], line["y2"]]], dtype=np.float32)
            boundary_key = f"{side}_{line_index}"
            line["side"] = side
            line["boundary_key"] = boundary_key
            line["pixel_count"] = len(segment)
            lines.append(line)
            boundary_points[boundary_key] = segment
        return edge, lines, len(raw_lines), boundary_points

    def analyze(self, semantic_mask, parallel_matrix):
        mask, component = self._select_ground_component(semantic_mask)
        edge, image_lines, raw_hough_segment_count, boundary_points = self._extract_side_lines(mask)
        bev_boundary_points = {
            side: self._project_points(points, parallel_matrix)
            for side, points in boundary_points.items()
        }
        bev_lines = []
        for line in image_lines:
            fitted = self._line_from_points(bev_boundary_points.get(line["boundary_key"], []))
            if fitted is not None:
                bev_lines.append(np.asarray([
                    [fitted["x1"], fitted["y1"]], [fitted["x2"], fitted["y2"]]
                ], dtype=np.float32))
        pair_measurements = []
        candidates = []
        min_mm = self.lane_width_mm * (1.0 - self.distance_tolerance_ratio)
        max_mm = self.lane_width_mm * (1.0 + self.distance_tolerance_ratio)
        for first_index in range(len(bev_lines)):
            for second_index in range(first_index + 1, len(bev_lines)):
                first, second = bev_lines[first_index], bev_lines[second_index]
                angle_deg = _segment_parallel_angle_deg(first, second)
                distance_px = _line_normal_distance(first, second)
                distance_mm = None if distance_px is None else distance_px / self.pixel_per_mm
                opposite_sides = image_lines[first_index]["side"] != image_lines[second_index]["side"]
                directions = []
                for segment in (first, second):
                    dx = float(segment[1][0] - segment[0][0])
                    dy = float(segment[1][1] - segment[0][1])
                    directions.append(np.degrees(np.arctan2(dx, -dy)))
                template_angle_deg = (float(np.mean(directions)) + 90.0) % 180.0 - 90.0
                template = template_positions_for_angle(template_angle_deg)
                first_id, second_id = self.template_pair_ids
                if first_id not in template or second_id not in template:
                    template = self.template_x_at_ref_mm
                expected_distance_mm = abs(float(template[first_id]) - float(template[second_id]))
                pair_min_mm = expected_distance_mm * (1.0 - self.distance_tolerance_ratio)
                pair_max_mm = expected_distance_mm * (1.0 + self.distance_tolerance_ratio)
                overlap_ok = _y_overlap_px(first, second)
                parallel_ok = angle_deg is not None and angle_deg <= self.parallel_tolerance_deg
                distance_ok = distance_mm is not None and pair_min_mm <= distance_mm <= pair_max_mm
                pair_measurements.append({
                    "i": first_index,
                    "j": second_index,
                    "parallel_angle_deg": angle_deg,
                    "normal_distance_mm": distance_mm,
                    "expected_distance_mm": expected_distance_mm,
                    "min_distance_mm": pair_min_mm,
                    "max_distance_mm": pair_max_mm,
                    "opposite_sides": opposite_sides,
                    "y_overlap_ok": overlap_ok,
                    "parallel_ok": parallel_ok,
                    "distance_ok": distance_ok,
                })
                if opposite_sides and overlap_ok and parallel_ok and distance_ok:
                    pair_midpoint = (np.asarray(first).mean(axis=0) + np.asarray(second).mean(axis=0)) * 0.5
                    candidates.append({
                        "i": first_index, "j": second_index,
                        "distance_mm": float(distance_mm),
                        "target_distance_mm": expected_distance_mm,
                        "parallel_angle_deg": float(angle_deg),
                        "mid_x_mm": float((pair_midpoint[0] - self.bev_width / 2.0) / self.pixel_per_mm),
                    })
        if pair_measurements:
            min_mm = min(item["min_distance_mm"] for item in pair_measurements)
            max_mm = max(item["max_distance_mm"] for item in pair_measurements)
        selected = min(
            candidates,
            key=lambda item: (abs(item["distance_mm"] - item["target_distance_mm"]),
                              abs(item["mid_x_mm"])),
        ) if candidates else None
        accepted_indices = set() if selected is None else {selected["i"], selected["j"]}
        return {
            "accepted": selected is not None,
            "fallback_reason": None if selected is not None else "no_parallel_distance_valid_pair",
            "component": component,
            "clean_mask": mask,
            "raw_hough_segment_count": raw_hough_segment_count,
            "raw_boundary_side_count": raw_hough_segment_count,
            "min_distance_mm": min_mm,
            "max_distance_mm": max_mm,
            "pair_measurements": pair_measurements,
            "edge_mask": edge,
            "boundary_points": boundary_points,
            "bev_boundary_points": bev_boundary_points,
            "image_lines": image_lines,
            "bev_lines": bev_lines,
            "accepted_image_lines": [image_lines[index] for index in sorted(accepted_indices)],
            "accepted_bev_lines": [bev_lines[index] for index in sorted(accepted_indices)],
            "rejected_bev_lines": [line for index, line in enumerate(bev_lines) if index not in accepted_indices],
            "pair": selected,
        }
