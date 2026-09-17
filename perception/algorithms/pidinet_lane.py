import math
from itertools import combinations
from itertools import combinations
from itertools import combinations

import cv2
import numpy as np


ANGLE_TOL_DEG = 3.0
NORMAL_DIST_TOL = 8.0
MIN_LINE_LENGTH = 25.0


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
    # OpenCV releases return either (N, 1, 4) or (N, 4).
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
    """Return the fraction of a source line covered by a binary semantic mask."""
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
    """Return the acute direction angle between two projected segments."""
    a1, a2 = np.asarray(seg_a[0], dtype=np.float64), np.asarray(seg_a[1], dtype=np.float64)
    b1, b2 = np.asarray(seg_b[0], dtype=np.float64), np.asarray(seg_b[1], dtype=np.float64)
    direction_a, direction_b = a2 - a1, b2 - b1
    length_a, length_b = np.linalg.norm(direction_a), np.linalg.norm(direction_b)
    if length_a < 1e-6 or length_b < 1e-6:
        return None
    cosine = abs(float(np.dot(direction_a, direction_b))) / (length_a * length_b)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _line_normal_distance(seg_a, seg_b):
    """Return the reference script's symmetric normal distance in BEV pixels."""
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
    """Whether every vehicle point lies in the canvas-closed lane strip.

    The two segment lines are extended to the BEV canvas boundary.  Their two
    infinite supporting lines partition the canvas into a lane strip and two
    exterior regions; this returns true only when the complete vehicle
    footprint is inside the strip.  A single point remains accepted for
    compatibility with geometry callers.
    """
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
    """Create the only 200 mm parallel partner that encloses the vehicle."""
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
    """Compute lane offset at the vehicle centre using infinite line extensions.

    The BEV cross-section is horizontal because the camera/body calibration has
    zero yaw.  Only the y-coordinate of the vehicle centre is fixed; both lane
    boundaries are extended analytically to intersect that cross-section.
    """
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
    """Strict a3d8 ground-IPM pair filter copied from the experiment script."""
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
    """Reference ordering: width nearest target, then midpoint nearest centre."""
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
        # Tests and callers may intentionally replace ``matrix`` (for example
        # with identity); honor that override for every profile.
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
        """Four body corners in BEV pixels, using the camera/body calibration."""
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
            # A virtual partner is physically meaningful for the lane-width
            # profile; wall outer corners are only accepted when both sides
            # are observed, avoiding an invented wall edge.
            if self.debug_pair_profile in ("auto", "lane"):
                profile = self.pair_profiles["lane"]
                self.candidate_bev_segments = ground_candidates
                # In an explicit lane-only debug run, do not reinterpret a
                # clearly observed wall-outer pair as a single lane edge.
                # Auto mode handles that pair in the wall_outer pass above.
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
        if self.final_pair is not None:
            for line, color, inferred in ((self.final_pair["left_img"], (0, 255, 0), self.final_pair["left_inferred"]),
                                          (self.final_pair["right_img"], (0, 0, 255), self.final_pair["right_inferred"])):
                segment = np.array([[line["x1"], line["y1"]], [line["x2"], line["y2"]]], dtype=np.float64)
                if inferred:
                    self._draw_dashed_segment(view, segment, color, 3)
                else:
                    self._draw_segment(view, segment, color, 3)
        if self.final_pair is not None and self.final_pair["inferred"]:
            cv2.putText(view, "dashed = inferred boundary", (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)
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


def _template_pair_weight(first, second):
    pair = frozenset((first, second))
    if pair == frozenset(("0", "1")):
        return 2.0
    if pair in (frozenset(("0", "2")), frozenset(("1", "3"))):
        return 1.5
    if pair in (frozenset(("2", "4")), frozenset(("3", "5"))):
        return 1.0
    return 0.5 if pair != frozenset(("4", "5")) else 0.3


class TemplateDistanceLaneSelector(GroundIPMLanePairSelector):
    """BPU-compatible six-line template matcher used by the real robot path.

    Projection and drawing are inherited from ``GroundIPMLanePairSelector``;
    only candidate grouping, template assignment, and pose validation differ.
    """

    def __init__(self, *args, max_match_rms_mm=15.0, min_confidence=0.30, **kwargs):
        super().__init__(*args, **kwargs)
        self.max_match_rms_mm = float(max_match_rms_mm)
        self.min_confidence = float(min_confidence)
        self.require_both_lane_lines = True

    def _pixel_to_ground(self, segment):
        segment = np.asarray(segment, dtype=np.float64)
        return np.asarray([
            [(point[0] - self.canvas_w / 2.0) / self.pixel_per_mm,
             (self.canvas_h - 1.0 - point[1]) / self.pixel_per_mm]
            for point in segment
        ], dtype=np.float64)

    @staticmethod
    def _line_params_ground(segment):
        first, second = np.asarray(segment[0]), np.asarray(segment[1])
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
        }

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

    def _robot_inside_corridor(self, delta, theta):
        half_width = float(self.vehicle_geometry.get("body_width_mm", 120.0)) / 2.0
        body_length = float(self.vehicle_geometry.get("body_length_mm", 142.0))
        camera_front = float(self.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        lateral = float(self.vehicle_geometry.get("camera_lateral_offset_mm", 0.0))
        x_left, x_right = lateral - half_width, lateral + half_width
        for y in (-camera_front, -camera_front - body_length):
            values = {}
            for lid in ("0", "1"):
                x_perp = TEMPLATE_LINE_X_MM[lid] + delta
                values[lid] = x_perp / math.cos(theta) + y * math.tan(theta)
            lo, hi = sorted((values["0"], values["1"]))
            if x_left < lo or x_right > hi:
                return False
        return True

    def _match_group(self, group, theta_candidate):
        enriched = []
        cos_theta, sin_theta = math.cos(theta_candidate), math.sin(theta_candidate)
        for item in group:
            params = item["params"]
            x_perp = params["mid_x"] * cos_theta - params["mid_y"] * sin_theta
            enriched.append({"item": item, "x_perp": x_perp})
        enriched.sort(key=lambda value: value["x_perp"], reverse=True)
        best = None
        for indexes in combinations(range(6), len(enriched)):
            ids = [TEMPLATE_LINE_ORDER[index] for index in indexes]
            if "0" not in ids or "1" not in ids:
                continue
            loss = 0.0
            for first in range(len(ids)):
                for second in range(first + 1, len(ids)):
                    detected = abs(enriched[first]["x_perp"] - enriched[second]["x_perp"])
                    expected = abs(TEMPLATE_LINE_X_MM[ids[first]] - TEMPLATE_LINE_X_MM[ids[second]])
                    difference = detected - expected
                    loss += _template_pair_weight(ids[first], ids[second]) * difference * difference
            numerator = sum(TEMPLATE_LINE_WEIGHTS[lid] * (enriched[i]["x_perp"] - TEMPLATE_LINE_X_MM[lid])
                            for i, lid in enumerate(ids))
            denominator = sum(TEMPLATE_LINE_WEIGHTS[lid] for lid in ids)
            delta = numerator / denominator
            residual = sum(TEMPLATE_LINE_WEIGHTS[lid] * (enriched[i]["x_perp"] - TEMPLATE_LINE_X_MM[lid] - delta) ** 2
                           for i, lid in enumerate(ids))
            rms = math.sqrt(residual / denominator)
            if rms > self.max_match_rms_mm or not self._robot_inside_corridor(delta, theta_candidate):
                continue
            candidate = {
                "assignment": {lid: enriched[i]["item"] for i, lid in enumerate(ids)},
                "ids": ids, "delta": float(delta), "loss": float(loss),
                "rms": float(rms), "theta": float(theta_candidate),
            }
            if best is None or candidate["loss"] < best["loss"]:
                best = candidate
        return best

    def _match(self, ground_items):
        groups = self._parallel_groups(ground_items)
        groups = [group for group in groups if 3 <= len(group) <= 6
                  and all(abs(ground_items[index]["params"]["theta"]) <= math.radians(45.0)
                          for index in group)]
        candidates = {round(ground_items[index]["params"]["theta"], 4)
                      for group in groups for index in group}
        candidates.update(math.radians(degree) for degree in (-10, -5, -2, 0, 2, 5, 10))
        best = None
        for theta in candidates:
            for group in groups:
                candidate = self._match_group([ground_items[index] for index in group], theta)
                if candidate is not None and (best is None or candidate["loss"] < best["loss"]):
                    best = candidate
        return best, len(groups)

    def _template_confidence(self, match):
        fit = max(0.0, 1.0 - match["rms"] / 10.0)
        precision = max(0.0, 1.0 - match["rms"] / 5.0)
        return 0.4 * fit + 0.3 * 1.0 + 0.3 * precision

    def analyze(self, source_lines, semantic_mask=None):
        self.selected_source_lines, self.detected_source_lines = [], []
        self.selected_bev_segments, self.final_pair = [], None
        self._source_candidates = list(source_lines)
        if self.semantic_gate and semantic_mask is None:
            state = self._empty_state("semantic_mask_required")
            state.update({"pair_profile": "template_distance", "template_match_count": 0})
            return state

        self.candidate_bev_segments = [self._project_line(line, self._matrix_for_profile("lane"))
                                       for line in source_lines]
        ground_items = []
        for index, segment in enumerate(self.candidate_bev_segments):
            ground = self._pixel_to_ground(segment)
            params = self._line_params_ground(ground)
            ground_items.append({"index": index, "segment": segment, "ground": ground, "params": params})
        match, group_count = self._match(ground_items)
        if match is None:
            state = self._empty_state("no_valid_template_match")
            state.update({"pair_profile": "template_distance", "template_match_count": group_count})
            return state

        confidence = self._template_confidence(match)
        if confidence < self.min_confidence:
            state = self._empty_state("low_template_confidence")
            state.update({"pair_profile": "template_distance", "template_match_count": group_count,
                          "template_confidence": confidence, "template_residual_mm": match["rms"]})
            return state

        assignment = match["assignment"]
        left, right = assignment["1"], assignment["0"]
        left_segment, right_segment = left["segment"], right["segment"]
        self.selected_bev_segments = [left_segment, right_segment]
        self.selected_source_lines = [source_lines[left["index"]], source_lines[right["index"]]]
        self.detected_source_lines = list(self.selected_source_lines)
        self.final_pair = {
            "left": left_segment, "right": right_segment,
            "left_img": self.selected_source_lines[0], "right_img": self.selected_source_lines[1],
            "left_inferred": False, "right_inferred": False, "inferred": False,
            "pair_profile": "template_distance", "target_mm": abs(TEMPLATE_LINE_X_MM["0"] - TEMPLATE_LINE_X_MM["1"]),
            "distance_mm": abs(TEMPLATE_LINE_X_MM["0"] - TEMPLATE_LINE_X_MM["1"]),
            "parallel_angle_deg": math.degrees(abs(left["params"]["theta"] - right["params"]["theta"])),
            "centerline": ((left_segment + right_segment) * 0.5).astype(np.float32),
            "template_match": match,
        }
        center_y = (self.canvas_h - 1.0 - self.vehicle_center[1]) / self.pixel_per_mm
        center_x = float(self.vehicle_geometry.get("camera_lateral_offset_mm", 0.0))
        theta = match["theta"]
        x0 = (TEMPLATE_LINE_X_MM["0"] + match["delta"]) / math.cos(theta) + center_y * math.tan(theta)
        x1 = (TEMPLATE_LINE_X_MM["1"] + match["delta"]) / math.cos(theta) + center_y * math.tan(theta)
        lane_center_x = (x0 + x1) * 0.5
        return {
            "pid_error_mm": float(lane_center_x - center_x), "crossroad_detected": False,
            "distance_to_crossroad_mm": -1.0, "lane_angle_rad": float(theta),
            "quality_score": float(confidence), "frame_dropped": False, "drop_reason": "",
            "duty_cycle": 0.0, "pair_profile": "template_distance",
            "pair_target_mm": self.final_pair["target_mm"], "pair_distance_mm": self.final_pair["distance_mm"],
            "template_match_count": len(match["ids"]), "template_confidence": float(confidence),
            "template_residual_mm": match["rms"], "template_delta_mm": match["delta"],
            "vis_points": {"blind_spot_pts": [], "normal_lane_pts": [tuple(point) for point in self.final_pair["centerline"]],
                           "crossroad_y": None, "left_boundary_pts": [tuple(point) for point in left_segment],
                           "right_boundary_pts": [tuple(point) for point in right_segment]},
        }
