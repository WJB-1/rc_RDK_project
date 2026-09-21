"""Binary semantic-mask lane boundary extraction and BEV distance gating."""

import math

import cv2
import numpy as np

from .line_geometry import find_valid_lane_pairs, select_best_pair


class SemanticLaneDetector:
    """Extract complete left/right mask boundaries before accepting a BEV lane pair."""

    def __init__(self, pixel_per_mm, bev_width, lane_width_mm,
                 distance_tolerance_mm=20.0, parallel_tolerance_deg=3.0,
                 min_segment_length_px=30, max_curve_residual_px=8.0):
        self.pixel_per_mm = float(pixel_per_mm)
        self.bev_width = int(bev_width)
        self.lane_width_mm = float(lane_width_mm)
        self.distance_tolerance_mm = float(distance_tolerance_mm)
        self.parallel_tolerance_deg = float(parallel_tolerance_deg)
        self.min_segment_length_px = int(min_segment_length_px)
        self.max_curve_residual_px = float(max_curve_residual_px)

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
        edge = cv2.Canny(mask, 50, 150)
        raw = cv2.HoughLinesP(
            edge, 1, np.pi / 180.0, threshold=18,
            minLineLength=self.min_segment_length_px, maxLineGap=24,
        )
        height, width = mask.shape[:2]
        sides = {"left": [], "right": []}
        for segment in raw.reshape(-1, 4) if raw is not None else []:
            x1, y1, x2, y2 = map(float, segment)
            dx, dy = x2 - x1, y2 - y1
            if abs(dy) < abs(dx) * 1.5:
                continue
            side = "left" if (x1 + x2) * 0.5 < width * 0.5 else "right"
            sides[side].extend(((x1, y1), (x2, y2)))
        lines = []
        for side, points in sides.items():
            line = self._line_from_points(points)
            if line is not None and line["curve_residual_px"] <= self.max_curve_residual_px:
                line["side"] = side
                lines.append(line)
        return edge, lines

    def analyze(self, semantic_mask, parallel_matrix):
        mask = np.where(np.asarray(semantic_mask) > 0, 255, 0).astype(np.uint8)
        edge, image_lines = self._extract_side_lines(mask)
        bev_lines = [self._project_line(line, parallel_matrix) for line in image_lines]
        candidates = find_valid_lane_pairs(
            bev_lines,
            self.pixel_per_mm,
            self.bev_width,
            min_mm=self.lane_width_mm - self.distance_tolerance_mm,
            max_mm=self.lane_width_mm + self.distance_tolerance_mm,
            parallel_tol_deg=self.parallel_tolerance_deg,
        )
        selected = select_best_pair(candidates, self.lane_width_mm)
        accepted_indices = set() if selected is None else {selected["i"], selected["j"]}
        return {
            "accepted": selected is not None,
            "fallback_reason": None if selected is not None else "no_parallel_distance_valid_pair",
            "edge_mask": edge,
            "image_lines": image_lines,
            "bev_lines": bev_lines,
            "accepted_image_lines": [image_lines[index] for index in sorted(accepted_indices)],
            "accepted_bev_lines": [bev_lines[index] for index in sorted(accepted_indices)],
            "rejected_bev_lines": [line for index, line in enumerate(bev_lines) if index not in accepted_indices],
            "pair": selected,
        }
