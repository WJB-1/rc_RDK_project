# perception/algorithms/lane/line_geometry.py
"""线段几何工具：平行判定、法向距离、pair 枚举、offset 计算。

常量统一从 .constants 取，模块内不再硬编码。
"""
import math

import numpy as np

from .constants import (
    PARALLEL_ANGLE_TOL_DEG, Y_OVERLAP_MIN_PX,
    PAIR_MIN_MM, PAIR_MAX_MM, PAIR_TARGET_MM,
)


def _segment_parallel_angle_deg(seg_a, seg_b):
    a1, a2 = np.asarray(seg_a[0], dtype=np.float64), np.asarray(seg_a[1], dtype=np.float64)
    b1, b2 = np.asarray(seg_b[0], dtype=np.float64), np.asarray(seg_b[1], dtype=np.float64)
    direction_a, direction_b = a2 - a1, b2 - b1
    length_a, length_b = np.linalg.norm(direction_a), np.linalg.norm(direction_b)
    if length_a < 1e-6 or length_b < 1e-6:
        return None
    cosine = abs(float(np.dot(direction_a, direction_b))) / (length_a * length_b)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _line_normal_distance(seg_a, seg_b):
    a1, a2 = np.asarray(seg_a[0], dtype=np.float64), np.asarray(seg_a[1], dtype=np.float64)
    b1, b2 = np.asarray(seg_b[0], dtype=np.float64), np.asarray(seg_b[1], dtype=np.float64)
    direction_a, direction_b = a2 - a1, b2 - b1
    length_a, length_b = np.linalg.norm(direction_a), np.linalg.norm(direction_b)
    if length_a < 1e-6 or length_b < 1e-6:
        return None
    normal_a = np.array([-direction_a[1], direction_a[0]]) / length_a
    normal_b = np.array([-direction_b[1], direction_b[0]]) / length_b
    midpoint_a, midpoint_b = (a1 + a2) * 0.5, (b1 + b2) * 0.5
    return float((abs(np.dot(midpoint_b - midpoint_a, normal_a))
                  + abs(np.dot(midpoint_a - midpoint_b, normal_b))) * 0.5)


def _y_overlap_px(seg_a, seg_b, min_overlap_px=Y_OVERLAP_MIN_PX):
    first_low, first_high = sorted((float(seg_a[0][1]), float(seg_a[1][1])))
    second_low, second_high = sorted((float(seg_b[0][1]), float(seg_b[1][1])))
    return min(first_high, second_high) - max(first_low, second_low) >= min_overlap_px


def vehicle_in_lane_closure(seg_a, seg_b, vehicle_point):
    first, second = np.asarray(seg_a, dtype=np.float64), np.asarray(seg_b, dtype=np.float64)
    direction = first[1] - first[0]
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return False
    normal = np.array([-direction[1], direction[0]]) / length
    first_position = float(np.dot(first[0], normal))
    second_position = float(np.dot(second[0], normal))
    vehicle_points = np.asarray(vehicle_point, dtype=np.float64).reshape(-1, 2)
    vehicle_positions = vehicle_points @ normal
    lower, upper = sorted((first_position, second_position))
    return bool(np.all((lower <= vehicle_positions) & (vehicle_positions <= upper)))


def infer_lane_pair_from_single_line(bev_segment, pixel_per_mm, lane_width_mm, vehicle_point):
    real = np.asarray(bev_segment, dtype=np.float64)
    vehicle_points = np.asarray(vehicle_point, dtype=np.float64).reshape(-1, 2)
    direction = real[1] - real[0]
    length = np.linalg.norm(direction)
    if length < 1e-6:
        return None
    normal = np.array([-direction[1], direction[0]]) / length
    offset_px = float(lane_width_mm) * float(pixel_per_mm)
    candidates = []
    for sign in (-1.0, 1.0):
        virtual = real + sign * offset_px * normal
        if not vehicle_in_lane_closure(real, virtual, vehicle_points):
            continue
        pair_midpoint = (real.mean(axis=0) + virtual.mean(axis=0)) * 0.5
        candidates.append({
            "real": real.astype(np.float32),
            "virtual": virtual.astype(np.float32),
            "distance_mm": float(lane_width_mm),
            "parallel_angle_deg": 0.0,
            "mid_x_mm": float((pair_midpoint[0] - vehicle_points[:, 0].mean()) / pixel_per_mm),
            "vehicle_enclosed": True,
        })
    return min(candidates, key=lambda item: abs(item["mid_x_mm"])) if candidates else None


def lane_offset_at_vehicle_cross_section(left_segment, right_segment, vehicle_center, pixel_per_mm):
    vehicle_x, vehicle_y = map(float, vehicle_center)

    def x_at_y(segment):
        first = np.asarray(segment[0], dtype=np.float64)
        second = np.asarray(segment[1], dtype=np.float64)
        delta_y = second[1] - first[1]
        if abs(delta_y) < 1e-6:
            raise ValueError("lane boundary is parallel to the vehicle cross-section")
        return float(first[0] + (vehicle_y - first[1]) * (second[0] - first[0]) / delta_y)

    left_x, right_x = x_at_y(left_segment), x_at_y(right_segment)
    lane_center_x = (left_x + right_x) * 0.5
    return (lane_center_x - vehicle_x) / float(pixel_per_mm), lane_center_x


def find_valid_lane_pairs(bev_segments, pixel_per_mm, bev_width, bev_height=None,
                          min_mm=PAIR_MIN_MM, max_mm=PAIR_MAX_MM,
                          parallel_tol_deg=PARALLEL_ANGLE_TOL_DEG,
                          vehicle_point=None):
    valid_pairs = []
    for first_index in range(len(bev_segments)):
        for second_index in range(first_index + 1, len(bev_segments)):
            first, second = bev_segments[first_index], bev_segments[second_index]
            if not _y_overlap_px(first, second):
                continue
            angle_deg = _segment_parallel_angle_deg(first, second)
            if angle_deg is None or angle_deg > parallel_tol_deg:
                continue
            distance_px = _line_normal_distance(first, second)
            if distance_px is None:
                continue
            distance_mm = distance_px / pixel_per_mm
            if distance_mm < min_mm or distance_mm > max_mm:
                continue
            if vehicle_point is not None and not vehicle_in_lane_closure(first, second, vehicle_point):
                continue
            first_midpoint = (np.asarray(first[0]) + np.asarray(first[1])) * 0.5
            second_midpoint = (np.asarray(second[0]) + np.asarray(second[1])) * 0.5
            pair_midpoint = (first_midpoint + second_midpoint) * 0.5
            valid_pairs.append({
                "i": first_index,
                "j": second_index,
                "distance_mm": float(distance_mm),
                "parallel_angle_deg": float(angle_deg),
                "mid_x_mm": float((pair_midpoint[0] - bev_width / 2.0) / pixel_per_mm),
                "mid_y_mm": float(((bev_height - 1 - pair_midpoint[1]) / pixel_per_mm)
                                  if bev_height is not None else 0.0),
                "vehicle_enclosed": vehicle_point is not None,
            })
    return valid_pairs


def select_best_pair(pairs, target_mm=PAIR_TARGET_MM):
    return min(pairs, key=lambda pair: (abs(pair["distance_mm"] - target_mm),
                                        abs(pair["mid_x_mm"]))) if pairs else None