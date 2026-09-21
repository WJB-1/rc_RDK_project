import math
from itertools import combinations

import cv2
import numpy as np

from .timing import block


ANGLE_TOL_DEG = 3.0
NORMAL_DIST_TOL = 8.0
MIN_LINE_LENGTH = 25.0


# ============================================================
# 新标定内外参：只用于把车道中心线投到新地面系算偏航角和偏移量
# 与车道检测的 BEV 矩阵无关
# ============================================================
GROUND_FX_PX = 1148.55348289962566
GROUND_FY_PX = 1148.55348289962566
GROUND_CX_PX = 947.8505552892422
GROUND_CY_PX = 567.5025328444085
GROUND_CAMERA_HEIGHT_MM = 158.17124739512468
GROUND_PITCH_DEG = 37.15759309700342
GROUND_CALIB_WIDTH = 1920
GROUND_CALIB_HEIGHT = 1080

# 车辆几何（算偏移量时用）
GROUND_CAMERA_FORWARD_MM = 60.0
GROUND_BODY_LENGTH_MM = 142.0
GROUND_CAMERA_LATERAL_OFFSET_MM = 0.0


def thin_binary(binary: np.ndarray) -> np.ndarray:
    binary = np.where(binary > 0, 255, 0).astype(np.uint8)
    ximgproc = getattr(cv2, "ximgproc", None)
    if ximgproc is not None and hasattr(ximgproc, "thinning"):
        return ximgproc.thinning(binary, thinningType=ximgproc.THINNING_ZHANGSUEN)

    skeleton = np.zeros_like(binary)
    current = binary.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while np.any(current):
        eroded = cv2.erode(current, kernel)
        opened = cv2.dilate(eroded, kernel)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(current, opened))
        current = eroded
    return skeleton


def postprocess_edge_probability(probability: np.ndarray, threshold: float = 0.35) -> np.ndarray:
    probability = np.asarray(probability, dtype=np.float32)
    binary = (probability >= float(threshold)).astype(np.uint8) * 255
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return thin_binary(binary)


def _line_params(line):
    x1, y1, x2, y2 = line["x1"], line["y1"], line["x2"], line["y2"]
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return None
    ux, uy = dx / length, dy / length
    if ux < 0 or (abs(ux) < 1e-9 and uy < 0):
        ux, uy = -ux, -uy
    nx, ny = -uy, ux
    return {
        "cx": (x1 + x2) * 0.5,
        "cy": (y1 + y2) * 0.5,
        "ux": ux,
        "uy": uy,
        "nx": nx,
        "ny": ny,
        "angle": math.degrees(math.atan2(uy, ux)) % 180.0,
        "length": length,
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
    }


def _can_merge(first, second, angle_tol_deg, normal_dist_tol):
    angle_delta = abs(first["angle"] - second["angle"])
    if angle_delta > 90.0:
        angle_delta = 180.0 - angle_delta
    if angle_delta > angle_tol_deg:
        return False
    first_distance = abs(
        (second["cx"] - first["cx"]) * first["nx"]
        + (second["cy"] - first["cy"]) * first["ny"]
    )
    second_distance = abs(
        (first["cx"] - second["cx"]) * second["nx"]
        + (first["cy"] - second["cy"]) * second["ny"]
    )
    return first_distance <= normal_dist_tol and second_distance <= normal_dist_tol


def _fit_line_pca(points):
    points = np.asarray(points, dtype=np.float64)
    mean = points.mean(axis=0)
    centered = points - mean
    covariance = centered.T @ centered / max(len(points), 1)
    _, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, -1]
    projections = centered @ direction
    first = mean + direction * projections.min()
    second = mean + direction * projections.max()
    return float(first[0]), float(first[1]), float(second[0]), float(second[1])


def _orientation(angle):
    if angle < 20.0 or angle > 160.0:
        return "horizontal"
    if 70.0 <= angle <= 110.0:
        return "vertical"
    return "diagonal"


def merge_lines(lines, angle_tol_deg=ANGLE_TOL_DEG, normal_dist_tol=NORMAL_DIST_TOL,
                min_length=MIN_LINE_LENGTH):
    params = [item for item in (_line_params(line) for line in lines) if item is not None]
    if not params:
        return []
    parent = list(range(len(params)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    for first_index, first in enumerate(params):
        for second_index in range(first_index + 1, len(params)):
            second = params[second_index]
            center_distance = math.hypot(first["cx"] - second["cx"], first["cy"] - second["cy"])
            bound = (first["length"] + second["length"]) * 0.5 + normal_dist_tol
            if center_distance <= bound * 4 and _can_merge(first, second, angle_tol_deg, normal_dist_tol):
                union(first_index, second_index)

    groups = {}
    for index in range(len(params)):
        groups.setdefault(find(index), []).append(index)

    merged = []
    for indices in groups.values():
        points = []
        for index in indices:
            points.extend(((params[index]["x1"], params[index]["y1"]),
                           (params[index]["x2"], params[index]["y2"])))
        if len(indices) == 1:
            first = params[indices[0]]
            x1, y1, x2, y2 = first["x1"], first["y1"], first["x2"], first["y2"]
        else:
            x1, y1, x2, y2 = _fit_line_pca(points)
        length = math.hypot(x2 - x1, y2 - y1)
        if length < min_length:
            continue
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0
        merged.append({
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "length": float(length),
            "angle_deg": float(angle),
            "orientation": _orientation(angle),
            "merged_from": len(indices),
        })
    return sorted(merged, key=lambda item: item["length"], reverse=True)


def detect_lines(edge_binary: np.ndarray):
    height, width = edge_binary.shape[:2]
    lines = cv2.HoughLinesP(
        edge_binary,
        rho=1,
        theta=np.pi / 180,
        threshold=30,
        minLineLength=int(max(30.0, min(width, height) * 0.08)),
        maxLineGap=max(8, int(min(width, height) * 0.02)),
    )
    if lines is None:
        return []
    result = []
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        length = math.hypot(float(x2 - x1), float(y2 - y1))
        if length < MIN_LINE_LENGTH:
            continue
        angle = math.degrees(math.atan2(float(y2 - y1), float(x2 - x1))) % 180.0
        result.append({
            "x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
            "length": length, "angle_deg": angle, "orientation": _orientation(angle),
        })
    return sorted(result, key=lambda item: item["length"], reverse=True)


def draw_lines(image: np.ndarray, lines, color=(0, 0, 255), thickness=2):
    result = image.copy()
    for line in lines:
        cv2.line(
            result,
            (round(line["x1"]), round(line["y1"])),
            (round(line["x2"]), round(line["y2"])),
            color,
            thickness,
            cv2.LINE_AA,
        )
    return result


def lines_to_mask(lines, shape, thickness=3):
    mask = np.zeros(shape[:2], dtype=np.uint8)
    for line in lines:
        cv2.line(
            mask,
            (round(line["x1"]), round(line["y1"])),
            (round(line["x2"]), round(line["y2"])),
            255,
            thickness,
            cv2.LINE_AA,
        )
    return mask


def line_mask_coverage(line, semantic_mask, thickness=3):
    if semantic_mask is None or semantic_mask.size == 0:
        return 0.0
    line_mask = np.zeros(semantic_mask.shape[:2], dtype=np.uint8)
    cv2.line(line_mask, (round(line["x1"]), round(line["y1"])),
             (round(line["x2"]), round(line["y2"])), 255, int(thickness), cv2.LINE_AA)
    line_pixels = line_mask > 0
    if not np.any(line_pixels):
        return 0.0
    covered = (np.asarray(semantic_mask) > 0) & line_pixels
    return float(np.count_nonzero(covered) / np.count_nonzero(line_pixels))


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


def _y_overlap_px(seg_a, seg_b, min_overlap_px=8.0):
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
        first, second = np.asarray(segment[0], dtype=np.float64), np.asarray(segment[1], dtype=np.float64)
        delta_y = second[1] - first[1]
        if abs(delta_y) < 1e-6:
            raise ValueError("lane boundary is parallel to the vehicle cross-section")
        return float(first[0] + (vehicle_y - first[1]) * (second[0] - first[0]) / delta_y)

    left_x, right_x = x_at_y(left_segment), x_at_y(right_segment)
    lane_center_x = (left_x + right_x) * 0.5
    return (lane_center_x - vehicle_x) / float(pixel_per_mm), lane_center_x


def find_valid_lane_pairs(bev_segments, pixel_per_mm, bev_width, bev_height=None,
                          min_mm=190.0, max_mm=210.0, parallel_tol_deg=1.5,
                          vehicle_point=None):
    valid_pairs = []
    for first_index in range(len(bev_segments)):
        for second_index in range(first_index + 1, len(bev_segments)):
            first, second = bev_segments[first_index], bev_segments[second_index]
            if not _y_overlap_px(first, second, min_overlap_px=8.0):
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


def select_best_pair(pairs, target_mm=200.0):
    return min(pairs, key=lambda pair: (abs(pair["distance_mm"] - target_mm),
                                        abs(pair["mid_x_mm"]))) if pairs else None


class GroundIPMLanePairSelector:
    """Ground-IPM projection and exact a3d8 lane-pair selection for PiDiNet lines."""

    def __init__(self, input_size, calibration_size, fx_px, fy_px, cx_px, cy_px,
                 camera_height_mm, pitch_deg, canvas_w, canvas_h, pixel_per_mm,
                 blind_spot_mm, lane_width_mm=200.0, vehicle_geometry=None,
                 pair_profiles=None, debug_pair_profile="auto", semantic_gate=False,
                 semantic_min_coverage=0.01, semantic_line_thickness=3):
        self.input_width, self.input_height = map(int, input_size)
        self.calibration_width, self.calibration_height = map(int, calibration_size)
        self.canvas_w, self.canvas_h_base = int(canvas_w), int(canvas_h)
        self.pixel_per_mm = float(pixel_per_mm)
        self.blind_spot_mm = float(blind_spot_mm)
        self.blind_spot_px = int(round(self.blind_spot_mm * self.pixel_per_mm))
        self.canvas_h = self.canvas_h_base + self.blind_spot_px
        self.lane_width_mm = float(lane_width_mm)
        self.vehicle_geometry = vehicle_geometry or {}
        default_profiles = {
            "lane": {"min_mm": 190.0, "max_mm": 210.0, "target_mm": self.lane_width_mm, "height_mm": 0.0},
            "wall_outer": {"min_mm": 236.0, "max_mm": 241.0, "target_mm": 236.0, "height_mm": 50.0},
        }
        if pair_profiles:
            for name, values in pair_profiles.items():
                if name in default_profiles and isinstance(values, dict):
                    default_profiles[name].update({
                        key: float(values[key]) for key in ("min_mm", "max_mm", "target_mm", "height_mm")
                        if key in values
                    })
        self.pair_profiles = default_profiles
        self.debug_pair_profile = str(debug_pair_profile or "auto").lower()
        if self.debug_pair_profile not in ("auto", *self.pair_profiles.keys()):
            self.debug_pair_profile = "auto"
        self.semantic_gate = bool(semantic_gate)
        self.semantic_min_coverage = float(semantic_min_coverage)
        self.semantic_line_thickness = int(semantic_line_thickness)
        self.matrix = self._build_matrix(fx_px, fy_px, cx_px, cy_px, camera_height_mm, pitch_deg)
        self._ground_matrix = self.matrix.copy()
        self._profile_matrices = {
            "lane": self._ground_matrix,
            "wall_outer": self._build_matrix(
                fx_px, fy_px, cx_px, cy_px,
                float(camera_height_mm) - self.pair_profiles["wall_outer"].get("height_mm", 50.0),
                pitch_deg,
            ),
        }
        self.selected_source_lines = []
        self.detected_source_lines = []
        self.selected_bev_segments = []
        self.candidate_bev_segments = []
        self.final_pair = None

    @property
    def canvas_size(self):
        return self.canvas_w, self.canvas_h

    def _build_matrix(self, fx_px, fy_px, cx_px, cy_px, height_mm, pitch_deg):
        scale_x = self.input_width / float(self.calibration_width)
        scale_y = self.input_height / float(self.calibration_height)
        intrinsic = np.array([
            [float(fx_px) * scale_x, 0.0, float(cx_px) * scale_x],
            [0.0, float(fy_px) * scale_y, float(cy_px) * scale_y],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        pitch_rad = math.radians(float(pitch_deg))
        exterior = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -math.sin(pitch_rad), float(height_mm) * math.cos(pitch_rad)],
            [0.0, math.cos(pitch_rad), float(height_mm) * math.sin(pitch_rad)],
        ], dtype=np.float64)
        scale = np.array([
            [self.pixel_per_mm, 0.0, self.canvas_w / 2.0],
            [0.0, -self.pixel_per_mm, self.canvas_h_base + self.blind_spot_px],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        return scale @ np.linalg.inv(intrinsic @ exterior)

    def _matrix_for_profile(self, profile_name):
        if not np.allclose(self.matrix, self._ground_matrix):
            return self.matrix
        return self._profile_matrices.get(profile_name, self.matrix)

    def _project_line(self, line, matrix=None):
        points = np.array([[[line["x1"], line["y1"]]], [[line["x2"], line["y2"]]]], dtype=np.float32)
        return cv2.perspectiveTransform(points, self.matrix if matrix is None else matrix).reshape(-1, 2)

    def _unproject_segment(self, segment, matrix=None):
        points = np.asarray(segment, dtype=np.float32).reshape(1, 2, 2)
        active_matrix = self.matrix if matrix is None else matrix
        image_points = cv2.perspectiveTransform(points, np.linalg.inv(active_matrix))[0]
        return {
            "x1": float(image_points[0][0]), "y1": float(image_points[0][1]),
            "x2": float(image_points[1][0]), "y2": float(image_points[1][1]),
        }

    @property
    def vehicle_point(self):
        return self.canvas_w / 2.0, float(self.canvas_h - 1)

    @property
    def vehicle_footprint(self):
        body_width = float(self.vehicle_geometry.get("body_width_mm", 120.0))
        body_length = float(self.vehicle_geometry.get("body_length_mm", 142.0))
        camera_forward = float(self.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        body_front = -camera_forward
        body_rear = body_front - body_length
        def to_bev(x_mm, y_mm):
            return (self.canvas_w / 2.0 + x_mm * self.pixel_per_mm,
                    self.canvas_h - 1 - y_mm * self.pixel_per_mm)
        return np.asarray([
            to_bev(-body_width / 2.0, body_front), to_bev(body_width / 2.0, body_front),
            to_bev(-body_width / 2.0, body_rear), to_bev(body_width / 2.0, body_rear),
        ], dtype=np.float64)

    @property
    def vehicle_center(self):
        body_length = float(self.vehicle_geometry.get("body_length_mm", 142.0))
        camera_forward = float(self.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        camera_lateral_offset = float(self.vehicle_geometry.get("camera_lateral_offset_mm", 0.0))
        centre_y_mm = -camera_forward - body_length * 0.5
        return (
            self.canvas_w / 2.0 + camera_lateral_offset * self.pixel_per_mm,
            self.canvas_h - 1 - centre_y_mm * self.pixel_per_mm,
        )

    def _empty_state(self, reason):
        return {
            "pid_error_mm": 0.0, "crossroad_detected": False, "distance_to_crossroad_mm": -1.0,
            "lane_angle_rad": 0.0, "quality_score": 0.0, "frame_dropped": True,
            "drop_reason": reason, "duty_cycle": 0.0,
            "pair_profile": None, "pair_target_mm": 0.0, "pair_distance_mm": 0.0,
            "semantic_boundary_coverage": 0.0,
            "vis_points": {"blind_spot_pts": [], "normal_lane_pts": [], "crossroad_y": None,
                           "left_boundary_pts": [], "right_boundary_pts": []},
        }

    @staticmethod
    def _forward_direction(segment):
        first, second = np.asarray(segment[0]), np.asarray(segment[1])
        direction = second - first if second[1] <= first[1] else first - second
        norm = np.linalg.norm(direction)
        return direction / norm if norm > 1e-6 else np.zeros(2, dtype=np.float64)

    def analyze(self, source_lines, semantic_mask=None):
        self.selected_source_lines, self.detected_source_lines = [], []
        self.selected_bev_segments, self.final_pair = [], None
        self.template_identities = None
        self.template_match = None
        self.template_conf = None
        self._source_candidates = list(source_lines)
        if semantic_mask is not None and semantic_mask.shape[:2] != (self.input_height, self.input_width):
            semantic_mask = cv2.resize(semantic_mask, (self.input_width, self.input_height),
                                       interpolation=cv2.INTER_NEAREST)
        source_coverages = [line_mask_coverage(line, semantic_mask, self.semantic_line_thickness)
                            for line in source_lines]
        if self.semantic_gate and semantic_mask is None:
            return self._empty_state("semantic_mask_required")
        ground_candidates = [self._project_line(line, self._matrix_for_profile("lane")) for line in source_lines]
        self.candidate_bev_segments = ground_candidates
        profile_names = ([self.debug_pair_profile] if self.debug_pair_profile != "auto"
                         else ["lane", "wall_outer"])
        chosen = None
        chosen_profile = None
        for profile_name in profile_names:
            profile = self.pair_profiles[profile_name]
            profile_candidates = [self._project_line(line, self._matrix_for_profile(profile_name)) for line in source_lines]
            self.candidate_bev_segments = profile_candidates
            pairs = find_valid_lane_pairs(
                profile_candidates, self.pixel_per_mm, self.canvas_w,
                self.canvas_h, min_mm=profile["min_mm"], max_mm=profile["max_mm"],
                parallel_tol_deg=1.5, vehicle_point=self.vehicle_footprint,
            )
            if self.semantic_gate:
                pairs = [pair for pair in pairs
                         if max(source_coverages[pair["i"]], source_coverages[pair["j"]])
                         >= self.semantic_min_coverage]
            chosen = select_best_pair(pairs, target_mm=profile["target_mm"])
            if chosen is not None:
                chosen_profile = profile_name
                chosen["pair_profile"] = profile_name
                chosen["target_mm"] = profile["target_mm"]
                break
        if chosen is None:
            inferred_candidates = []
            if self.debug_pair_profile in ("auto", "lane"):
                profile = self.pair_profiles["lane"]
                self.candidate_bev_segments = ground_candidates
                if self.debug_pair_profile == "lane":
                    wall = self.pair_profiles["wall_outer"]
                    wall_pairs = find_valid_lane_pairs(
                        self.candidate_bev_segments, self.pixel_per_mm, self.canvas_w,
                        self.canvas_h, min_mm=wall["min_mm"], max_mm=wall["max_mm"],
                        parallel_tol_deg=1.5, vehicle_point=self.vehicle_footprint,
                    )
                    if wall_pairs:
                        return self._empty_state("wall_outer_pair_requires_wall_profile")
                for index, segment in enumerate(self.candidate_bev_segments):
                    inferred = infer_lane_pair_from_single_line(
                        segment, self.pixel_per_mm, profile["target_mm"], self.vehicle_footprint
                    )
                    if inferred is None:
                        continue
                    virtual_image_line = self._unproject_segment(
                        inferred["virtual"], self._matrix_for_profile("lane")
                    )
                    virtual_coverage = line_mask_coverage(
                        virtual_image_line, semantic_mask, self.semantic_line_thickness
                    )
                    if self.semantic_gate and max(source_coverages[index], virtual_coverage) < self.semantic_min_coverage:
                        continue
                    inferred_candidates.append({**inferred, "i": index,
                                                "virtual_coverage": virtual_coverage,
                                                "pair_profile": "lane",
                                                "target_mm": profile["target_mm"]})
            if not inferred_candidates:
                return self._empty_state("no_enclosing_lane_pair")
            chosen = min(inferred_candidates, key=lambda item: (abs(item["mid_x_mm"]), -source_lines[item["i"]].get("length", 0.0)))
            first_index = chosen["i"]
            second_index = first_index
            first_segment, second_segment = chosen["real"], chosen["virtual"]
            first_img, second_img = source_lines[first_index], self._unproject_segment(
                second_segment, self._matrix_for_profile("lane")
            )
            first_inferred, second_inferred = False, True
            selected_coverages = [
                source_coverages[first_index],
                float(chosen["virtual_coverage"]),
            ]
            self.detected_source_lines = [first_img]
        else:
            first_index, second_index = chosen["i"], chosen["j"]
            first_segment, second_segment = self.candidate_bev_segments[first_index], self.candidate_bev_segments[second_index]
            first_img, second_img = source_lines[first_index], source_lines[second_index]
            first_inferred, second_inferred = False, False
            selected_coverages = [source_coverages[first_index], source_coverages[second_index]]
            self.detected_source_lines = [first_img, second_img]

        first_mid_x, second_mid_x = first_segment[:, 0].mean(), second_segment[:, 0].mean()
        if first_mid_x <= second_mid_x:
            left_segment, right_segment = first_segment, second_segment
            left_img, right_img = first_img, second_img
            left_inferred, right_inferred = first_inferred, second_inferred
        else:
            left_segment, right_segment = second_segment, first_segment
            left_img, right_img = second_img, first_img
            left_inferred, right_inferred = second_inferred, first_inferred
        self.selected_source_lines = [left_img, right_img]
        self.selected_bev_segments = [left_segment, right_segment]
        self.final_pair = {**chosen, "left": left_segment, "right": right_segment,
                           "left_img": left_img, "right_img": right_img,
                           "left_inferred": left_inferred, "right_inferred": right_inferred,
                           "inferred": left_inferred or right_inferred}
        self.final_pair["pair_profile"] = chosen_profile or chosen.get("pair_profile", "lane")
        self.final_pair["target_mm"] = float(chosen.get("target_mm", self.pair_profiles[self.final_pair["pair_profile"]]["target_mm"]))
        self.final_pair["centerline"] = ((left_segment + right_segment) * 0.5).astype(np.float32)
        self.final_pair["semantic_boundary_coverage"] = max(selected_coverages) if source_lines else 0.0
        centre_direction = self._forward_direction(left_segment) + self._forward_direction(right_segment)
        lane_angle_rad = math.atan2(float(centre_direction[0]), float(-centre_direction[1]))
        try:
            offset_mm, lane_centre_x = lane_offset_at_vehicle_cross_section(
                left_segment, right_segment, self.vehicle_center, self.pixel_per_mm
            )
        except ValueError:
            return self._empty_state("boundary_parallel_to_vehicle_cross_section")
        self.final_pair["vehicle_cross_section_y"] = self.vehicle_center[1]
        self.final_pair["lane_centre_x"] = lane_centre_x
        width_score = max(0.0, 1.0 - abs(chosen["distance_mm"] - self.final_pair["target_mm"]) / 10.0)
        angle_score = max(0.0, 1.0 - chosen["parallel_angle_deg"] / 1.5)
        centre_points = ((left_segment + right_segment) * 0.5).astype(float)
        quality_score = (width_score + angle_score) * 0.5
        if self.final_pair["inferred"]:
            quality_score *= 0.7
        return {
            "pid_error_mm": offset_mm, "crossroad_detected": False,
            "distance_to_crossroad_mm": -1.0, "lane_angle_rad": lane_angle_rad,
            "quality_score": quality_score, "frame_dropped": False,
            "drop_reason": "", "duty_cycle": 0.0,
            "lane_pair_mode": "inferred_single_boundary" if self.final_pair["inferred"] else "detected_pair",
            "pair_profile": self.final_pair["pair_profile"],
            "pair_target_mm": self.final_pair["target_mm"],
            "pair_distance_mm": float(chosen["distance_mm"]),
            "semantic_boundary_coverage": self.final_pair["semantic_boundary_coverage"],
            "vis_points": {"blind_spot_pts": [], "normal_lane_pts": [tuple(point) for point in centre_points],
                           "crossroad_y": None, "left_boundary_pts": [tuple(point) for point in left_segment],
                           "right_boundary_pts": [tuple(point) for point in right_segment]},
        }

    @staticmethod
    def _draw_segment(canvas, segment, color, thickness):
        first = tuple(np.round(segment[0]).astype(int))
        second = tuple(np.round(segment[1]).astype(int))
        height, width = canvas.shape[:2]
        if not (-500 <= first[0] <= width + 500 and -500 <= first[1] <= height + 500):
            return
        if not (-500 <= second[0] <= width + 500 and -500 <= second[1] <= height + 500):
            return
        cv2.line(canvas, first, second, color, thickness, cv2.LINE_AA)

    @staticmethod
    def _draw_dashed_segment(canvas, segment, color, thickness, dash_px=12):
        first, second = np.asarray(segment[0], dtype=np.float64), np.asarray(segment[1], dtype=np.float64)
        direction = second - first
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return
        direction /= length
        for start in np.arange(0.0, length, dash_px * 2.0):
            end = min(start + dash_px, length)
            cv2.line(canvas, tuple(np.round(first + direction * start).astype(int)),
                     tuple(np.round(first + direction * end).astype(int)), color, thickness, cv2.LINE_AA)

    def draw_original_view(self, processing_frame):
        view = processing_frame.copy()
        for line in self._source_candidates:
            cv2.line(view, tuple(np.round([line["x1"], line["y1"]]).astype(int)),
                     tuple(np.round([line["x2"], line["y2"]]).astype(int)), (255, 100, 0), 1, cv2.LINE_AA)

        if self.template_identities is not None:
            y_low = 0.0
            y_high = (self.canvas_h - 1.0) / self.pixel_per_mm
            lane_matrix = self._matrix_for_profile("lane")
            for lid in ALL_LINE_IDS:
                info = self.template_identities[lid]
                color = TEMPLATE_LINE_COLORS[lid]
                ground_first, ground_second = self._inferred_line_endpoints(
                    info["x_at_ref_mm"], info["theta"], y_low, y_high, self.y_ref_mm,
                )
                bev_segment = self._ground_to_bev_segment(ground_first, ground_second)
                image_segment = self._unproject_segment(bev_segment, lane_matrix)
                segment = np.asarray(
                    [[image_segment["x1"], image_segment["y1"]],
                     [image_segment["x2"], image_segment["y2"]]], dtype=np.float64
                )
                first = tuple(np.round(segment[0]).astype(int))
                if info["matched"]:
                    self._draw_segment(view, segment, color, 3)
                else:
                    self._draw_dashed_segment(view, segment, color, 1)
                cv2.putText(view, lid, (first[0] + 5, first[1] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

            if self.final_pair is not None:
                centre_bev = self.final_pair.get("centerline")
                if centre_bev is not None:
                    centre_img = self._unproject_segment(centre_bev, lane_matrix)
                    centre_segment = np.asarray(
                        [[centre_img["x1"], centre_img["y1"]],
                         [centre_img["x2"], centre_img["y2"]]], dtype=np.float64
                    )
                    self._draw_segment(view, centre_segment, (255, 0, 255), 3)
                    mid = centre_segment.mean(axis=0).astype(int)
                    cv2.putText(view, "center", (mid[0] + 6, mid[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2, cv2.LINE_AA)
        return view

    def _draw_coordinate_system(self, view):
        height, width = view.shape[:2]
        origin_x, origin_y = width // 2, height - 1
        tick_px = max(1, int(round(10.0 * self.pixel_per_mm)))
        for x in range(origin_x, width, tick_px):
            cv2.line(view, (x, 0), (x, height), (42, 42, 42), 1)
        for x in range(origin_x - tick_px, -1, -tick_px):
            cv2.line(view, (x, 0), (x, height), (42, 42, 42), 1)
        for y in range(origin_y, -1, -tick_px):
            cv2.line(view, (0, y), (width, y), (42, 42, 42), 1)
        cv2.arrowedLine(view, (0, origin_y), (width - 4, origin_y), (235, 235, 235), 1,
                        cv2.LINE_AA, tipLength=0.02)
        cv2.arrowedLine(view, (origin_x, origin_y), (origin_x, 4), (235, 235, 235), 1,
                        cv2.LINE_AA, tipLength=0.03)
        cv2.putText(view, "X (mm)", (width - 55, origin_y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(view, "Y fwd (mm)", (origin_x + 6, 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (235, 235, 235), 1, cv2.LINE_AA)

    def _draw_vehicle(self, view):
        height = view.shape[0]
        def to_px(x_mm, y_mm):
            return (int(round(self.canvas_w / 2.0 + x_mm * self.pixel_per_mm)),
                    int(round(height - 1 - y_mm * self.pixel_per_mm)))

        body_length = float(self.vehicle_geometry.get("body_length_mm", 142.0))
        body_width = float(self.vehicle_geometry.get("body_width_mm", 120.0))
        camera_forward = float(self.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        camera_width = float(self.vehicle_geometry.get("camera_width_mm", 31.0))
        camera_length = float(self.vehicle_geometry.get("camera_length_mm", 60.0))
        body_front, body_rear = -camera_forward, -camera_forward - body_length
        cv2.rectangle(view, to_px(-body_width / 2.0, body_front), to_px(body_width / 2.0, body_rear),
                      (0, 190, 255), 2)
        camera_first, camera_second = to_px(-camera_width / 2.0, 0.0), to_px(camera_width / 2.0, -camera_length)
        for first, second in ((camera_first, (camera_second[0], camera_first[1])),
                              ((camera_second[0], camera_first[1]), camera_second),
                              (camera_second, (camera_first[0], camera_second[1])),
                              ((camera_first[0], camera_second[1]), camera_first)):
            cv2.line(view, first, second, (180, 180, 180), 1, cv2.LINE_AA)

    def draw_bev_view(self):
        view = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
        self._draw_coordinate_system(view)
        for segment in self.candidate_bev_segments:
            self._draw_segment(view, segment, (200, 80, 0), 1)
        if self.final_pair is not None:
            for segment, color, inferred in ((self.final_pair["left"], (0, 255, 0), self.final_pair["left_inferred"]),
                                              (self.final_pair["right"], (0, 0, 255), self.final_pair["right_inferred"])):
                if inferred:
                    self._draw_dashed_segment(view, segment, color, 3)
                else:
                    self._draw_segment(view, segment, color, 3)
        self._draw_vehicle(view)
        cv2.circle(view, tuple(np.round(self.vehicle_point).astype(int)), 4, (0, 255, 255), -1)
        if self.final_pair is not None:
            cross_y = int(round(self.final_pair["vehicle_cross_section_y"]))
            centre_x = int(round(self.final_pair["lane_centre_x"]))
            vehicle_x = int(round(self.vehicle_center[0]))
            cv2.line(view, (0, cross_y), (self.canvas_w - 1, cross_y), (0, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(view, (centre_x, cross_y), 4, (255, 255, 0), -1)
            cv2.circle(view, (vehicle_x, cross_y), 4, (0, 255, 255), -1)
        if self.final_pair is None:
            info = f"cand={len(self.candidate_bev_segments)} | pair: none"
        else:
            info = (f"cand={len(self.candidate_bev_segments)} | profile={self.final_pair['pair_profile']} "
                    f"d={self.final_pair['distance_mm']:.1f}mm "
                    f"(|d-{self.final_pair['target_mm']:.0f}|={abs(self.final_pair['distance_mm'] - self.final_pair['target_mm']):.1f}) | "
                    f"ang={self.final_pair['parallel_angle_deg']:.2f}deg")
        cv2.putText(view, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(view, "cand=blue | final: L=green R=red | dashed=inferred", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        return view


TEMPLATE_LINE_X_MM = {
    "0": 99.771, "1": -99.789,
    "2": 144.157, "3": -144.721,
    "4": 170.861, "5": -171.389,
}
TEMPLATE_LINE_ORDER = ("4", "2", "0", "1", "3", "5")
TEMPLATE_LINE_WEIGHTS = {"0": 1.0, "1": 1.0, "2": 0.5, "3": 0.5, "4": 0.2, "5": 0.2}
LANE_LINE_IDS = ("0", "1")
WALL_LINE_IDS = ("2", "3", "4", "5")
ALL_LINE_IDS = ("0", "1", "2", "3", "4", "5")
TEMPLATE_LINE_COLORS = {
    "0": (0, 200, 255), "1": (0, 100, 255),
    "2": (0, 255, 0), "3": (0, 180, 0),
    "4": (255, 0, 255), "5": (180, 0, 180),
}


class TemplateDistanceLaneSelector(GroundIPMLanePairSelector):
    """BPU-compatible six-line template matcher used by the real robot path.

    车道检测（BEV、模板匹配、身份识别）走旧 IPM 参数。
    偏航角和偏移量单独走新标定内外参，在"新地面系"里算。
    """

    def __init__(self, *args,
                 max_match_rms_mm=15.0,
                 min_confidence=0.30,
                 line_count_reward_lambda=5.0,
                 max_forward_line_angle_deg=45.0,
                 match_tie_loss_epsilon=1e-6,
                 conf_fit_scale=10.0,
                 conf_lane_precision_scale=5.0,
                 conf_weights=(0.4, 0.3, 0.3),
                 inferred_conf_factor=0.5,
                 y_ref_mm=500.0,
                 require_both_lane_lines=False,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.max_match_rms_mm = float(max_match_rms_mm)
        self.min_confidence = float(min_confidence)
        self.line_count_reward_lambda = float(line_count_reward_lambda)
        self.max_forward_line_angle_deg = float(max_forward_line_angle_deg)
        self.match_tie_loss_epsilon = float(match_tie_loss_epsilon)
        self.conf_fit_scale = float(conf_fit_scale)
        self.conf_lane_precision_scale = float(conf_lane_precision_scale)
        self.conf_weights = tuple(float(weight) for weight in conf_weights)
        self.inferred_conf_factor = float(inferred_conf_factor)
        self.y_ref_mm = float(y_ref_mm)
        self.require_both_lane_lines = bool(require_both_lane_lines)
        self.template_identities = None
        self.template_match = None
        self.template_conf = None

    # ------------------------------------------------------------------
    # Ground reconstruction（旧 BEV 用）
    # ------------------------------------------------------------------
    def _pixel_to_ground(self, segment):
        segment = np.asarray(segment, dtype=np.float64)
        return np.asarray([
            [(point[0] - self.canvas_w / 2.0) / self.pixel_per_mm,
             (self.canvas_h - 1.0 - point[1]) / self.pixel_per_mm]
            for point in segment
        ], dtype=np.float64)

    @staticmethod
    def _line_params_ground(segment):
        first = np.asarray(segment[0], dtype=np.float64)
        second = np.asarray(segment[1], dtype=np.float64)
        dx, dy = second - first
        theta = math.atan2(float(dx), float(dy))
        if theta > math.pi / 2:
            theta -= math.pi
        elif theta <= -math.pi / 2:
            theta += math.pi
        return {
            "theta": theta,
            "mid_x": float((first[0] + second[0]) * 0.5),
            "mid_y": float((first[1] + second[1]) * 0.5),
            "length_mm": math.hypot(float(dx), float(dy)),
            "p1": (float(first[0]), float(first[1])),
            "p2": (float(second[0]), float(second[1])),
        }

    @staticmethod
    def _normalize_theta(theta):
        theta = float(theta)
        while theta > math.pi / 2:
            theta -= math.pi
        while theta <= -math.pi / 2:
            theta += math.pi
        return theta

    @staticmethod
    def _line_x_at_ref(params, y_ref):
        return params["mid_x"] + (y_ref - params["mid_y"]) * math.tan(params["theta"])

    @staticmethod
    def _parallel_groups(lines, tolerance_deg=3.0):
        parent = list(range(len(lines)))
        tolerance = math.radians(tolerance_deg)

        def find(index):
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        def union(first, second):
            first, second = find(first), find(second)
            if first != second:
                parent[second] = first

        for first in range(len(lines)):
            for second in range(first + 1, len(lines)):
                distance = abs((lines[first]["params"]["theta"] - lines[second]["params"]["theta"] + math.pi / 2) % math.pi - math.pi / 2)
                if distance <= tolerance:
                    union(first, second)
        groups = {}
        for index in range(len(lines)):
            groups.setdefault(find(index), []).append(index)
        return list(groups.values())

    @staticmethod
    def _estimate_group_theta(group):
        sin2 = 0.0
        cos2 = 0.0
        for item in group:
            params = item["params"]
            weight = max(float(params.get("length_mm", 0.0)), 1.0)
            theta = TemplateDistanceLaneSelector._normalize_theta(params["theta"])
            sin2 += weight * math.sin(2.0 * theta)
            cos2 += weight * math.cos(2.0 * theta)
        if abs(sin2) < 1e-12 and abs(cos2) < 1e-12:
            return 0.0
        return TemplateDistanceLaneSelector._normalize_theta(0.5 * math.atan2(sin2, cos2))

    def _robot_center_inside_inferred_lane_corridor(self, delta, group_theta):
        body_length = float(self.vehicle_geometry.get("body_length_mm", 142.0))
        camera_front = float(self.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        lateral = float(self.vehicle_geometry.get("camera_lateral_offset_mm", 0.0))
        y_center = -camera_front - body_length * 0.5
        tan_theta = math.tan(self._normalize_theta(group_theta))
        lane_x = {
            lid: TEMPLATE_LINE_X_MM[lid] + delta + (y_center - self.y_ref_mm) * tan_theta
            for lid in LANE_LINE_IDS
        }
        lo, hi = sorted((lane_x["0"], lane_x["1"]))
        return lo <= lateral <= hi, {
            "robot_center_x_mm": lateral,
            "robot_center_y_mm": y_center,
            "lane_0_x_mm": lane_x["0"],
            "lane_1_x_mm": lane_x["1"],
        }

    # ------------------------------------------------------------------
    # Template matching（旧 BEV 用，不动）
    # ------------------------------------------------------------------
    def _match_group_to_template(self, group):
        n = len(group)
        if n < 3 or n > 10:
            return None

        enriched = []
        for item in group:
            x_ref = self._line_x_at_ref(item["params"], self.y_ref_mm)
            enriched.append({"item": item, "x_at_ref": x_ref})
        enriched.sort(key=lambda entry: entry["x_at_ref"], reverse=True)
        sorted_x = [entry["x_at_ref"] for entry in enriched]
        sorted_items = [entry["item"] for entry in enriched]

        group_theta = self._estimate_group_theta(group)
        weights = TEMPLATE_LINE_WEIGHTS
        reward_lambda = self.line_count_reward_lambda
        best = None
        second_loss = None
        max_fit_lines = min(6, n)
        for n_fit in range(3, max_fit_lines + 1):
            for line_idx in combinations(range(n), n_fit):
                selected_x = [sorted_x[i] for i in line_idx]
                selected_items = [sorted_items[i] for i in line_idx]
                for sub_idx in combinations(range(6), n_fit):
                    assigned_ids = [TEMPLATE_LINE_ORDER[i] for i in sub_idx]
                    n_lanes = sum(1 for lid in assigned_ids if lid in LANE_LINE_IDS)
                    if self.require_both_lane_lines and n_lanes != len(LANE_LINE_IDS):
                        continue
                    denominator = sum(float(weights[lid]) for lid in assigned_ids)
                    if denominator <= 0:
                        continue
                    delta = sum(
                        float(weights[lid]) * (selected_x[i] - float(TEMPLATE_LINE_X_MM[lid]))
                        for i, lid in enumerate(assigned_ids)
                    ) / denominator
                    fit_loss = sum(
                        float(weights[lid])
                        * (selected_x[i] - float(TEMPLATE_LINE_X_MM[lid]) - delta) ** 2
                        for i, lid in enumerate(assigned_ids)
                    ) / denominator
                    rms = math.sqrt(fit_loss)
                    coverage_reward = sum(float(weights[lid]) for lid in assigned_ids)
                    loss = fit_loss - reward_lambda * coverage_reward

                    robot_inside, pose = self._robot_center_inside_inferred_lane_corridor(
                        delta, group_theta,
                    )
                    if not robot_inside or rms > self.max_match_rms_mm:
                        continue

                    candidate = {
                        "assignment": {lid: selected_items[i] for i, lid in enumerate(assigned_ids)},
                        "assigned_ids": list(assigned_ids),
                        "delta": float(delta),
                        "loss": float(loss),
                        "fit_loss": float(fit_loss),
                        "coverage_reward": float(coverage_reward),
                        "rms": float(rms),
                        "group_theta": float(group_theta),
                        "n_matched": int(n_fit),
                        "robot_center_inside": True,
                        "pose": pose,
                    }
                    if best is None or candidate["loss"] < best["loss"]:
                        if best is not None:
                            second_loss = best["loss"]
                        best = candidate
                    elif second_loss is None or candidate["loss"] < second_loss:
                        second_loss = candidate["loss"]

        if best is None:
            return None
        loss_gap = math.inf if second_loss is None else second_loss - best["loss"]
        best["second_best_loss"] = second_loss
        best["loss_gap"] = loss_gap
        best["near_tie"] = loss_gap <= self.match_tie_loss_epsilon
        return best

    def _match_all_groups(self, ground_items):
        groups = self._parallel_groups(ground_items)
        max_theta = math.radians(self.max_forward_line_angle_deg)
        valid_groups = [
            group for group in groups
            if 3 <= len(group) <= 10
            and all(abs(self._normalize_theta(ground_items[index]["params"]["theta"])) <= max_theta
                    for index in group)
        ]
        best = None
        for group in valid_groups:
            candidate = self._match_group_to_template([ground_items[index] for index in group])
            if candidate is not None and (best is None or candidate["loss"] < best["loss"]):
                best = candidate
        return best, len(groups)

    def _compute_confidence(self, match):
        rms = match["rms"]
        fit_score = max(0.0, 1.0 - rms / self.conf_fit_scale)
        total_weight = sum(float(TEMPLATE_LINE_WEIGHTS[lid]) for lid in ALL_LINE_IDS)
        count_score = match["coverage_reward"] / total_weight if total_weight else 0.0
        precision_score = max(0.0, 1.0 - rms / self.conf_lane_precision_scale)
        n_lanes = sum(1 for lid in match["assignment"] if lid in LANE_LINE_IDS)
        w_fit, w_count, w_prec = self.conf_weights
        confidence = w_fit * fit_score + w_count * count_score + w_prec * precision_score
        return {
            "confidence": float(confidence),
            "fit_score": float(fit_score),
            "count_score": float(count_score),
            "precision_score": float(precision_score),
            "n_lanes": int(n_lanes),
        }

    def _build_identity_result(self, assignment, delta, group_theta, confidence):
        inferred_theta = self._normalize_theta(group_theta)
        result = {}
        for lid in ALL_LINE_IDS:
            template_x_ref = TEMPLATE_LINE_X_MM[lid] + delta
            if lid in assignment:
                item = assignment[lid]
                params = item["params"]
                result[lid] = {
                    "matched": True,
                    "confidence": float(confidence),
                    "x_at_ref_mm": self._line_x_at_ref(params, self.y_ref_mm),
                    "template_x_at_ref_mm": template_x_ref,
                    "theta": params["theta"],
                    "length_mm": params["length_mm"],
                    "p1": params["p1"],
                    "p2": params["p2"],
                    "item": item,
                }
            else:
                result[lid] = {
                    "matched": False,
                    "confidence": float(confidence * self.inferred_conf_factor),
                    "x_at_ref_mm": template_x_ref,
                    "template_x_at_ref_mm": template_x_ref,
                    "theta": inferred_theta,
                    "length_mm": 0.0,
                    "p1": None,
                    "p2": None,
                    "item": None,
                }
        return result

    @staticmethod
    def _inferred_line_endpoints(x_at_ref, theta, y_low, y_high, y_ref):
        theta = TemplateDistanceLaneSelector._normalize_theta(theta)
        tan_t = math.tan(theta)
        x1 = x_at_ref + (y_low - y_ref) * tan_t
        x2 = x_at_ref + (y_high - y_ref) * tan_t
        return (x1, y_low), (x2, y_high)

    def _ground_to_bev_segment(self, p1, p2):
        def to_pixel(x_mm, y_mm):
            return (self.canvas_w / 2.0 + x_mm * self.pixel_per_mm,
                    self.canvas_h - 1.0 - y_mm * self.pixel_per_mm)
        return np.asarray([to_pixel(*p1), to_pixel(*p2)], dtype=np.float32)

    def _identity_bev_segment(self, identity):
        y_low = 0.0
        y_high = (self.canvas_h - 1.0) / self.pixel_per_mm
        p1, p2 = self._inferred_line_endpoints(
            identity["x_at_ref_mm"], identity["theta"], y_low, y_high, self.y_ref_mm,
        )
        return self._ground_to_bev_segment(p1, p2)

    # ------------------------------------------------------------------
    # 新地面系几何：用 GROUND_* 内外参，与车道检测的 BEV 无关
    # 输入输出都在 input_size（512×384）图像坐标
    # ------------------------------------------------------------------
    def _scale_new_intrinsics(self):
        sx = self.input_width / float(GROUND_CALIB_WIDTH)
        sy = self.input_height / float(GROUND_CALIB_HEIGHT)
        return (
            GROUND_FX_PX * sx, GROUND_FY_PX * sy,
            GROUND_CX_PX * sx, GROUND_CY_PX * sy,
        )

    def _image_to_new_ground(self, u, v):
        """图像点（input_size 坐标）→ 新地面系 (X_mm, Y_mm)。"""
        fx, fy, cx, cy = self._scale_new_intrinsics()
        pitch_rad = math.radians(GROUND_PITCH_DEG)
        sp, cp = math.sin(pitch_rad), math.cos(pitch_rad)
        k = (v - cy) / fy
        denom = sp + k * cp
        if abs(denom) < 1e-9:
            return None
        y_mm = GROUND_CAMERA_HEIGHT_MM * (cp - k * sp) / denom
        z_cam = cp * y_mm + GROUND_CAMERA_HEIGHT_MM * sp
        if z_cam <= 1e-6:
            return None
        x_mm = (u - cx) / fx * z_cam
        return float(x_mm), float(y_mm)

    def _new_ground_to_image(self, x_mm, y_mm):
        """新地面系 (X_mm, Y_mm) → 图像点（input_size 坐标）。"""
        fx, fy, cx, cy = self._scale_new_intrinsics()
        pitch_rad = math.radians(GROUND_PITCH_DEG)
        sp, cp = math.sin(pitch_rad), math.cos(pitch_rad)
        denom = y_mm * cp + GROUND_CAMERA_HEIGHT_MM * sp
        if abs(denom) < 1e-9:
            return None
        k = (GROUND_CAMERA_HEIGHT_MM * cp - y_mm * sp) / denom
        z_cam = cp * y_mm + GROUND_CAMERA_HEIGHT_MM * sp
        if z_cam <= 1e-6:
            return None
        v = cy + k * fy
        u = cx + x_mm * fx / z_cam
        return float(u), float(v)

    def _source_line_to_new_ground(self, line):
        """原图 Hough 线 → 新地面系采样点列表 [(X, Y), ...]。"""
        if line is None:
            return []
        n = 40
        ts = np.linspace(0.0, 1.0, n)
        pts = []
        for t in ts:
            u = line["x1"] + t * (line["x2"] - line["x1"])
            v = line["y1"] + t * (line["y2"] - line["y1"])
            g = self._image_to_new_ground(u, v)
            if g is not None:
                pts.append(g)
        return pts

    @staticmethod
    def _fit_new_ground_centerline(pts):
        """新地面系点 → x = a*y + b。返回 (a, b) 或 None。"""
        if len(pts) < 5:
            return None
        ys = np.array([p[1] for p in pts], dtype=np.float64)
        xs = np.array([p[0] for p in pts], dtype=np.float64)
        keep = (ys > 50.0) & (ys < 8000.0)
        if keep.sum() < 5:
            return None
        a, b = np.polyfit(ys[keep], xs[keep], 1)
        return float(a), float(b)

    # ------------------------------------------------------------------
    # Frame analysis
    # ------------------------------------------------------------------
    def _ground_point_to_image(self, x_mm: float, y_mm: float):
        """旧地面 → 图像（供 draw_original_view 用）。"""
        bev_u = self.canvas_w / 2.0 + x_mm * self.pixel_per_mm
        bev_v = self.canvas_h - 1.0 - y_mm * self.pixel_per_mm
        bev_pt = np.array([[[bev_u, bev_v]]], dtype=np.float32)
        lane_matrix = self._matrix_for_profile("lane")
        img_pt = cv2.perspectiveTransform(bev_pt, np.linalg.inv(lane_matrix))[0, 0]
        return float(img_pt[0]), float(img_pt[1])

    def _image_point_to_ground(self, px: float, py: float):
        img_pt = np.array([[[px, py]]], dtype=np.float32)
        lane_matrix = self._matrix_for_profile("lane")
        bev_pt = cv2.perspectiveTransform(img_pt, lane_matrix)[0, 0]
        x_mm = (float(bev_pt[0]) - self.canvas_w / 2.0) / self.pixel_per_mm
        y_mm = (self.canvas_h - 1.0 - float(bev_pt[1])) / self.pixel_per_mm
        return x_mm, y_mm

    def analyze(self, source_lines, semantic_mask=None):
        self.selected_source_lines, self.detected_source_lines = [], []
        self.selected_bev_segments, self.final_pair = [], None
        self._source_candidates = list(source_lines)
        if self.semantic_gate and semantic_mask is None:
            state = self._empty_state("semantic_mask_required")
            state.update({"pair_profile": "template_distance", "template_match_count": 0})
            return state

        # ---------------- 车道检测：旧 BEV 全流程，不动 ----------------
        with block("lane.project_lines_bev"):
            self.candidate_bev_segments = [
                self._project_line(line, self._matrix_for_profile("lane"))
                for line in source_lines
            ]

        with block("lane.pixel_to_ground"):
            ground_items = []
            for index, segment in enumerate(self.candidate_bev_segments):
                ground = self._pixel_to_ground(segment)
                params = self._line_params_ground(ground)
                ground_items.append({
                    "index": index, "segment": segment, "ground": ground, "params": params,
                })

        with block("lane.template_match"):
            match, group_count = self._match_all_groups(ground_items)

        if match is None:
            state = self._empty_state("no_valid_template_match")
            state.update({"pair_profile": "template_distance", "template_match_count": group_count})
            return state

        with block("lane.compute_confidence"):
            conf = self._compute_confidence(match)
            confidence = conf["confidence"]
        if confidence < self.min_confidence:
            state = self._empty_state("low_template_confidence")
            state.update({
                "pair_profile": "template_distance",
                "template_match_count": group_count,
                "template_confidence": confidence,
                "template_residual_mm": match["rms"],
            })
            return state

        with block("lane.build_identity"):
            identities = self._build_identity_result(
                match["assignment"], match["delta"], match["group_theta"], confidence,
            )
            for lid in ALL_LINE_IDS:
                identities[lid]["source_line"] = (
                    source_lines[identities[lid]["item"]["index"]] if identities[lid]["matched"] else None
                )
        self.template_identities = identities
        self.template_match = match
        self.template_conf = conf

        with block("lane.identity_bev_segment"):
            left_info = identities["1"]
            right_info = identities["0"]
            left_segment = self._identity_bev_segment(left_info)
            right_segment = self._identity_bev_segment(right_info)
        self.selected_bev_segments = [left_segment, right_segment]
        self.selected_source_lines = [left_info["source_line"], right_info["source_line"]]
        self.detected_source_lines = [line for line in self.selected_source_lines if line is not None]

        with block("lane.build_final_pair"):
            parallel_angle = abs((left_info["theta"] - right_info["theta"] + math.pi / 2) % math.pi - math.pi / 2)
            centre_points = ((left_segment + right_segment) * 0.5).astype(np.float32)
            self.final_pair = {
                "left": left_segment, "right": right_segment,
                "left_img": left_info["source_line"], "right_img": right_info["source_line"],
                "left_inferred": not left_info["matched"], "right_inferred": not right_info["matched"],
                "inferred": (not left_info["matched"]) or (not right_info["matched"]),
                "pair_profile": "template_distance",
                "target_mm": abs(TEMPLATE_LINE_X_MM["0"] - TEMPLATE_LINE_X_MM["1"]),
                "distance_mm": abs(TEMPLATE_LINE_X_MM["0"] - TEMPLATE_LINE_X_MM["1"]),
                "parallel_angle_deg": math.degrees(parallel_angle),
                "centerline": centre_points,
                "vehicle_cross_section_y": self.vehicle_center[1],
                "lane_centre_x": float(centre_points[:, 0].mean()),
                "template_match": match,
            }

        # ============================================================
        # 偏航角和偏移量：新地面系，直接走原图 Hough 线 → 新内外参
        # ============================================================
        left_src = left_info["source_line"]
        right_src = right_info["source_line"]

        with block("lane.source_to_new_ground"):
            left_ground_pts = self._source_line_to_new_ground(left_src) if left_src is not None else []
            right_ground_pts = self._source_line_to_new_ground(right_src) if right_src is not None else []

        with block("lane.fit_new_ground"):
            left_fit = self._fit_new_ground_centerline(left_ground_pts)
            right_fit = self._fit_new_ground_centerline(right_ground_pts)

        if left_fit is not None and right_fit is not None:
            a_c = 0.5 * (left_fit[0] + right_fit[0])
            b_c = 0.5 * (left_fit[1] + right_fit[1])
            theta_source = "new_ground_pair"
            used_left = True
            used_right = True
        elif left_fit is not None:
            a_c, b_c = left_fit
            theta_source = "new_ground_left_only"
            used_left = True
            used_right = False
        elif right_fit is not None:
            a_c, b_c = right_fit
            theta_source = "new_ground_right_only"
            used_left = False
            used_right = True
        else:
            a_c, b_c = 0.0, 0.0
            theta_source = "bev_fallback"
            used_left = False
            used_right = False

        with block("lane.theta_offset"):
            if used_left or used_right:
                theta_ground = math.atan(a_c)
                body_length = float(self.vehicle_geometry.get("body_length_mm", GROUND_BODY_LENGTH_MM))
                camera_forward = float(self.vehicle_geometry.get(
                    "camera_forward_of_body_front_mm", GROUND_CAMERA_FORWARD_MM))
                camera_lateral_offset = float(self.vehicle_geometry.get(
                    "camera_lateral_offset_mm", GROUND_CAMERA_LATERAL_OFFSET_MM))
                y_vehicle_center_mm = -camera_forward - body_length * 0.5
                offset_mm = a_c * y_vehicle_center_mm + b_c - camera_lateral_offset
                lane_angle_rad = theta_ground
            else:
                # BEV fallback：用旧 BEV 中心线方向兜底，避免 lane_angle_rad 未绑定
                centre_direction = (self._forward_direction(left_segment)
                                    + self._forward_direction(right_segment))
                lane_angle_rad = math.atan2(
                    float(centre_direction[0]), float(-centre_direction[1]))
                theta_ground = lane_angle_rad
                offset_mm = 0.0

        with block("lane.new_ground_to_image"):
            center_img_p1 = None
            center_img_p2 = None
            if used_left or used_right:
                ys_plot = np.linspace(200.0, 3000.0, 20)
                pts_img = []
                for y in ys_plot:
                    x = a_c * y + b_c
                    p = self._new_ground_to_image(x, y)
                    if p is not None:
                        pts_img.append(p)
                if len(pts_img) >= 2:
                    center_img_p1 = pts_img[0]
                    center_img_p2 = pts_img[-1]
            if center_img_p1 is None or center_img_p2 is None:
                center_img_p1 = (self.input_width / 2.0, self.input_height - 1)
                center_img_p2 = (self.input_width / 2.0, 0.0)

        self.final_pair["center_image_p1"] = center_img_p1
        self.final_pair["center_image_p2"] = center_img_p2
        self.final_pair["theta_source"] = theta_source
        self.final_pair["lane_angle_deg"] = math.degrees(lane_angle_rad)
        self.final_pair["offset_mm"] = float(offset_mm)
        # 可视化缓存
        self._last_new_ground_left_pts = list(left_ground_pts)
        self._last_new_ground_right_pts = list(right_ground_pts)
        self._last_new_ground_center_fit = (a_c, b_c) if (used_left or used_right) else None
        self._last_new_ground_source = theta_source

        return {
            "pid_error_mm": float(offset_mm),
            "crossroad_detected": False,
            "distance_to_crossroad_mm": -1.0,
            "lane_angle_rad": float(lane_angle_rad),
            "lane_angle_source": theta_source,
            "quality_score": confidence,
            "frame_dropped": False,
            "drop_reason": "",
            "duty_cycle": 0.0,
            "pair_profile": "template_distance",
            "pair_target_mm": self.final_pair["target_mm"],
            "pair_distance_mm": self.final_pair["distance_mm"],
            "template_match_count": match["n_matched"],
            "template_confidence": confidence,
            "template_residual_mm": match["rms"],
            "template_delta_mm": match["delta"],
            "template_fit_loss": match["fit_loss"],
            "template_coverage_reward": match["coverage_reward"],
            "template_n_lanes": conf["n_lanes"],
            "vis_points": {
                "blind_spot_pts": [],
                "normal_lane_pts": [tuple(point) for point in centre_points],
                "crossroad_y": None,
                "left_boundary_pts": [tuple(point) for point in left_segment],
                "right_boundary_pts": [tuple(point) for point in right_segment],
            },
        }

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def draw_bev_view(self):
        view = np.zeros((self.canvas_h, self.canvas_w, 3), dtype=np.uint8)
        self._draw_coordinate_system(view)
        for segment in self.candidate_bev_segments:
            self._draw_segment(view, segment, (200, 80, 0), 1)

        if self.template_identities is not None:
            for lid in ALL_LINE_IDS:
                info = self.template_identities[lid]
                color = TEMPLATE_LINE_COLORS[lid]
                segment = self._identity_bev_segment(info)
                if info["matched"]:
                    self._draw_segment(view, segment, color, 3)
                else:
                    self._draw_dashed_segment(view, segment, color, 1)
                mid_x = int(round(self.canvas_w / 2.0 + info["x_at_ref_mm"] * self.pixel_per_mm))
                mid_y = int(round(self.canvas_h - 1.0 - self.y_ref_mm * self.pixel_per_mm))
                tag = f"{lid}" if info["matched"] else f"{lid}*"
                cv2.putText(view, tag, (mid_x + 4, mid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        self._draw_vehicle(view)
        cv2.circle(view, tuple(np.round(self.vehicle_point).astype(int)), 4, (0, 255, 255), -1)

        if self.template_match is not None:
            delta = self.template_match.get("delta", 0.0)
            rms = self.template_match.get("rms", 0.0)
            n_matched = self.template_match.get("n_matched", 0)
            conf = self.template_conf.get("confidence", 0.0) if self.template_conf else 0.0
            n_lanes = self.template_conf.get("n_lanes", 0) if self.template_conf else 0
            for index, line in enumerate((
                f"delta={delta:+.1f} mm | rms={rms:.2f} mm",
                f"conf={conf:.2f} | matched={n_matched} | lanes={n_lanes}",
            )):
                cv2.putText(view, line, (10, 25 + 20 * index),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(view, f"cand={len(self.candidate_bev_segments)} | pair: none",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(view, "solid=matched | dashed=inferred", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        return view

    def draw_original_view(self, processing_frame):
        view = processing_frame.copy()
        for line in self._source_candidates:
            cv2.line(view, tuple(np.round([line["x1"], line["y1"]]).astype(int)),
                     tuple(np.round([line["x2"], line["y2"]]).astype(int)),
                     (255, 100, 0), 1, cv2.LINE_AA)

        if self.template_identities is not None:
            y_low = 0.0
            y_high = (self.canvas_h - 1.0) / self.pixel_per_mm
            lane_matrix = self._matrix_for_profile("lane")
            for lid in ALL_LINE_IDS:
                info = self.template_identities[lid]
                color = TEMPLATE_LINE_COLORS[lid]
                ground_first, ground_second = self._inferred_line_endpoints(
                    info["x_at_ref_mm"], info["theta"], y_low, y_high, self.y_ref_mm,
                )
                bev_segment = self._ground_to_bev_segment(ground_first, ground_second)
                image_segment = self._unproject_segment(bev_segment, lane_matrix)
                segment = np.asarray(
                    [[image_segment["x1"], image_segment["y1"]],
                     [image_segment["x2"], image_segment["y2"]]], dtype=np.float64
                )
                first = tuple(np.round(segment[0]).astype(int))
                if info["matched"]:
                    self._draw_segment(view, segment, color, 3)
                else:
                    self._draw_dashed_segment(view, segment, color, 1)
                cv2.putText(view, lid, (first[0] + 5, first[1] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        if self.final_pair is not None:
            p1 = self.final_pair.get("center_image_p1")
            p2 = self.final_pair.get("center_image_p2")
            if p1 is not None and p2 is not None:
                self._draw_segment(
                    view,
                    np.asarray([p1, p2], dtype=np.float64),
                    (255, 0, 255), 3,
                )
                mid = (int((p1[0] + p2[0]) * 0.5), int((p1[1] + p2[1]) * 0.5))
                cv2.putText(view, "center(new)",
                            (mid[0] + 6, mid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 0, 255), 2, cv2.LINE_AA)

        h, w = view.shape[:2]
        cx = w // 2
        dash_len, gap_len = 12, 8
        y = 0
        while y < h:
            y_end = min(y + dash_len, h)
            cv2.line(view, (cx, y), (cx, y_end), (0, 255, 255), 1, cv2.LINE_AA)
            y += dash_len + gap_len
        cv2.putText(view, "image center", (cx + 5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

        h_view, w_view = view.shape[:2]
        if self.final_pair is not None:
            angle_deg = float(self.final_pair.get("lane_angle_deg", 0.0))
            offset_mm = float(self.final_pair.get("offset_mm", 0.0))
            source = self.final_pair.get("theta_source", "?")
            for i, text in enumerate((
                f"yaw = {angle_deg:+.3f} deg ({source})",
                f"offset = {offset_mm:+.1f} mm",
            )):
                cv2.putText(view, text, (10, h_view - 40 + i * 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
        return view

    # ------------------------------------------------------------------
    # 新地面系可视化
    # ------------------------------------------------------------------
    def render_new_ground_bev(self):
        """真正的新地面系 BEV：
          - 把左右身份线的原图 Hough 线用新内外参投到新地面系
          - 画中心线拟合结果
          - x=0 黄虚线，x=±100 灰线，车辆示意
        视野 x ∈ [-600, 600] mm，y ∈ [0, 3000] mm
        """
        X_MIN, X_MAX = -600.0, 600.0
        Y_MIN, Y_MAX = 0.0, 3000.0
        SCALE = 0.5
        MARGIN = 40
        W = int(round((X_MAX - X_MIN) * SCALE)) + 2 * MARGIN
        H = int(round((Y_MAX - Y_MIN) * SCALE)) + 2 * MARGIN

        def g2b(x, y):
            u = MARGIN + (x - X_MIN) * SCALE
            v = H - MARGIN - (y - Y_MIN) * SCALE
            return int(round(u)), int(round(v))

        canvas = np.zeros((H, W, 3), dtype=np.uint8)
        canvas[:] = (28, 28, 28)

        # 网格
        for x_ref in range(-600, 601, 100):
            color = (80, 80, 80) if x_ref % 200 == 0 else (42, 42, 42)
            u1 = g2b(x_ref, Y_MIN)[0]
            u2 = g2b(x_ref, Y_MAX)[0]
            cv2.line(canvas, (u1, MARGIN), (u1, H - MARGIN), color, 1, cv2.LINE_AA)
        for y_ref in range(0, int(Y_MAX) + 1, 200):
            color = (80, 80, 80) if y_ref % 1000 == 0 else (42, 42, 42)
            v = g2b(0.0, y_ref)[1]
            cv2.line(canvas, (MARGIN, v), (W - MARGIN, v), color, 1, cv2.LINE_AA)
            cv2.putText(canvas, f"{y_ref}", (4, v + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (170, 170, 170), 1, cv2.LINE_AA)

        # x=0 黄虚线
        u0, _ = g2b(0.0, 0.0)
        yy = MARGIN
        while yy < H - MARGIN:
            ye = min(yy + 12, H - MARGIN)
            cv2.line(canvas, (u0, yy), (u0, ye), (0, 255, 255), 1, cv2.LINE_AA)
            yy += 20
        # x=±100 参考
        for x_ref in (-100.0, 100.0):
            uu, _ = g2b(x_ref, 0.0)
            cv2.line(canvas, (uu, MARGIN), (uu, H - MARGIN), (90, 90, 90), 1, cv2.LINE_AA)

        # 车辆示意
        u0c, v0c = g2b(0.0, 0.0)
        cv2.rectangle(canvas, (u0c - 25, v0c - 12), (u0c + 25, v0c + 4),
                      (0, 190, 255), 1)

        # 从 self 上取本帧新地面采样
        left_pts = getattr(self, "_last_new_ground_left_pts", []) or []
        right_pts = getattr(self, "_last_new_ground_right_pts", []) or []
        center_fit = getattr(self, "_last_new_ground_center_fit", None)

        def draw_pts(pts, color, thickness=2):
            if len(pts) < 2:
                return
            for i in range(len(pts) - 1):
                p1 = g2b(pts[i][0], pts[i][1])
                p2 = g2b(pts[i + 1][0], pts[i + 1][1])
                cv2.line(canvas, p1, p2, color, thickness, cv2.LINE_AA)

        draw_pts(left_pts, (0, 255, 0))
        draw_pts(right_pts, (0, 0, 255))

        if center_fit is not None:
            a_c, b_c = center_fit
            ys = np.linspace(50.0, Y_MAX - 100.0, 20)
            cpts = [(a_c * y + b_c, y) for y in ys]
            draw_pts(cpts, (255, 0, 255), 2)
            mid = cpts[len(cpts) // 2]
            um, vm = g2b(*mid)
            yaw_deg = math.degrees(math.atan(a_c))
            cv2.putText(canvas, f"yaw={yaw_deg:+.3f}",
                        (um + 6, vm),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 0, 255), 1, cv2.LINE_AA)

        src = getattr(self, "_last_new_ground_source", "?")
        cv2.putText(canvas, f"NEW GROUND BEV  ({src})", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(canvas, "L=green R=red C=magenta  x=0=yellow  x=+-100=gray",
                    (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (200, 200, 200), 1, cv2.LINE_AA)
        return canvas

    def render_original_with_ground_centerline(self, processing_frame):
        """原图 + 匹配到的 left/right Hough 线 + 新地面中心线投回原图。"""
        if processing_frame is None:
            return None

        view = processing_frame.copy()
        H_img, W_img = view.shape[:2]
        sx = W_img / float(self.input_width)
        sy = H_img / float(self.input_height)

        # 1) Hough 候选（蓝）
        for line in self._source_candidates:
            p1 = (int(round(line["x1"] * sx)), int(round(line["y1"] * sy)))
            p2 = (int(round(line["x2"] * sx)), int(round(line["y2"] * sy)))
            cv2.line(view, p1, p2, (255, 100, 0), 1, cv2.LINE_AA)

        # 2) 左右身份线（绿/红）
        if self.final_pair is not None:
            for line, color in ((self.final_pair.get("left_img"), (0, 255, 0)),
                                (self.final_pair.get("right_img"), (0, 0, 255))):
                if line is None:
                    continue
                p1 = (int(round(line["x1"] * sx)), int(round(line["y1"] * sy)))
                p2 = (int(round(line["x2"] * sx)), int(round(line["y2"] * sy)))
                cv2.line(view, p1, p2, color, 3, cv2.LINE_AA)

            # 3) 新地面中心线回投影（品红）
            p1 = self.final_pair.get("center_image_p1")
            p2 = self.final_pair.get("center_image_p2")
            if p1 is not None and p2 is not None:
                q1 = (int(round(p1[0] * sx)), int(round(p1[1] * sy)))
                q2 = (int(round(p2[0] * sx)), int(round(p2[1] * sy)))
                cv2.line(view, q1, q2, (255, 0, 255), 3, cv2.LINE_AA)
                mid = (int((q1[0] + q2[0]) * 0.5), int((q1[1] + q2[1]) * 0.5))
                cv2.putText(view, "center(new-ground)",
                            (mid[0] + 6, mid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 0, 255), 2, cv2.LINE_AA)

        # 4) 图像中心竖线（黄）
        cx_img = W_img // 2
        yy = 0
        while yy < H_img:
            ye = min(yy + 12, H_img)
            cv2.line(view, (cx_img, yy), (cx_img, ye), (0, 255, 255), 1, cv2.LINE_AA)
            yy += 20

        # 5) 底部文字
        if self.final_pair is not None:
            angle_deg = float(self.final_pair.get("lane_angle_deg", 0.0))
            offset_mm = float(self.final_pair.get("offset_mm", 0.0))
            source = self.final_pair.get("theta_source", "?")
            for i, text in enumerate((
                f"yaw = {angle_deg:+.3f} deg ({source})",
                f"offset = {offset_mm:+.1f} mm",
            )):
                cv2.putText(view, text, (10, H_img - 40 + i * 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
        return view