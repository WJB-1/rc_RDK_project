"""Binary semantic-mask boundary extraction and BEV distance gating."""

import math

import cv2
import numpy as np

from .angle_template import template_positions_for_angle
from .line_geometry import (
    _line_normal_distance, _segment_parallel_angle_deg, _y_overlap_px,
    find_valid_lane_pairs, select_best_pair,
)


class SemanticLaneDetector:
    """Extract complete left/right mask boundaries before accepting a BEV lane pair."""

    def __init__(self, pixel_per_mm, bev_width, lane_width_mm,
                 distance_tolerance_mm=20.0, parallel_tolerance_deg=3.0,
                 min_segment_length_px=30, max_curve_residual_px=8.0,
                 template_pair_ids=("0", "1"), template_x_at_ref_mm=None,
                 edge_row_step_px=4, edge_row_search_px=12):
        self.pixel_per_mm = float(pixel_per_mm)
        self.bev_width = int(bev_width)
        self.lane_width_mm = float(lane_width_mm)
        self.distance_tolerance_mm = float(distance_tolerance_mm)
        self.parallel_tolerance_deg = float(parallel_tolerance_deg)
        self.min_segment_length_px = int(min_segment_length_px)
        self.max_curve_residual_px = float(max_curve_residual_px)
        self.template_pair_ids = tuple(template_pair_ids)
        self.template_x_at_ref_mm = dict(template_x_at_ref_mm or {})
        self.edge_row_step_px = max(1, int(edge_row_step_px))
        self.edge_row_search_px = max(0, int(edge_row_search_px))

    @staticmethod
    def _select_ground_component(mask, bottom_margin_px=10):
        binary = np.where(np.asarray(mask) > 0, 255, 0).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
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

    def _extract_side_lines(self, mask):
        binary = np.asarray(mask, dtype=np.uint8) > 0
        edge = np.zeros_like(np.asarray(mask, dtype=np.uint8))
        sides = {"left": [], "right": []}
        height = binary.shape[0]
        for y in range(0, height, self.edge_row_step_px):
            xs = np.flatnonzero(binary[y])
            if not len(xs) and self.edge_row_search_px:
                lower = max(0, y - self.edge_row_search_px)
                upper = min(height, y + self.edge_row_search_px + 1)
                for candidate_y in range(lower, upper):
                    candidate_xs = np.flatnonzero(binary[candidate_y])
                    if len(candidate_xs):
                        xs = candidate_xs
                        y = candidate_y
                        break
            if not len(xs):
                continue
            left_x, right_x = int(xs[0]), int(xs[-1])
            edge[y, left_x] = 255
            edge[y, right_x] = 255
            sides["left"].append((left_x, y))
            if right_x != left_x:
                sides["right"].append((right_x, y))
        lines = []
        for side, points in sides.items():
            line = self._line_from_points(points)
            if line is not None and line["curve_residual_px"] <= self.max_curve_residual_px:
                line["side"] = side
                lines.append(line)
        return edge, lines, sum(bool(points) for points in sides.values())

    def analyze(self, semantic_mask, parallel_matrix):
        mask, component = self._select_ground_component(semantic_mask)
        edge, image_lines, raw_hough_segment_count = self._extract_side_lines(mask)
        bev_lines = [self._project_line(line, parallel_matrix) for line in image_lines]
        pair_measurements = []
        min_mm = self.lane_width_mm - self.distance_tolerance_mm
        max_mm = self.lane_width_mm + self.distance_tolerance_mm
        if len(bev_lines) >= 1:
            directions = []
            for line in bev_lines:
                dx = float(line[1][0] - line[0][0])
                dy = float(line[1][1] - line[0][1])
                directions.append(np.degrees(np.arctan2(dx, -dy)))
            angle_deg = float(np.mean(directions))
            angle_deg = (angle_deg + 90.0) % 180.0 - 90.0
            template = template_positions_for_angle(angle_deg)
            first_id, second_id = self.template_pair_ids
            if first_id not in template or second_id not in template:
                template = self.template_x_at_ref_mm
            expected_distance_mm = abs(float(template[first_id]) - float(template[second_id]))
            min_mm = expected_distance_mm - self.distance_tolerance_mm
            max_mm = expected_distance_mm + self.distance_tolerance_mm
        for first_index in range(len(bev_lines)):
            for second_index in range(first_index + 1, len(bev_lines)):
                first, second = bev_lines[first_index], bev_lines[second_index]
                angle_deg = _segment_parallel_angle_deg(first, second)
                distance_px = _line_normal_distance(first, second)
                distance_mm = None if distance_px is None else distance_px / self.pixel_per_mm
                pair_measurements.append({
                    "i": first_index,
                    "j": second_index,
                    "parallel_angle_deg": angle_deg,
                    "normal_distance_mm": distance_mm,
                    "y_overlap_ok": _y_overlap_px(first, second),
                    "parallel_ok": angle_deg is not None and angle_deg <= self.parallel_tolerance_deg,
                    "distance_ok": distance_mm is not None and min_mm <= distance_mm <= max_mm,
                })
        candidates = find_valid_lane_pairs(
            bev_lines,
            self.pixel_per_mm,
            self.bev_width,
            min_mm=min_mm,
            max_mm=max_mm,
            parallel_tol_deg=self.parallel_tolerance_deg,
        )
        selected = select_best_pair(candidates, self.lane_width_mm)
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
            "image_lines": image_lines,
            "bev_lines": bev_lines,
            "accepted_image_lines": [image_lines[index] for index in sorted(accepted_indices)],
            "accepted_bev_lines": [bev_lines[index] for index in sorted(accepted_indices)],
            "rejected_bev_lines": [line for index, line in enumerate(bev_lines) if index not in accepted_indices],
            "pair": selected,
        }
