# perception/algorithms/lane/ground_ipm_selector.py
"""Ground-IPM 车道对选择器（a3d8）。

投影逻辑全部由 GroundIPMProjector 承担。
本类只负责：候选对枚举 → 单线推理 → 最优对选择 → 车道状态输出。
"""
import math

import cv2
import numpy as np

from .constants import PARALLEL_ANGLE_TOL_DEG
from .ground_ipm_projector import GroundIPMProjector
from .line_geometry import (
    find_valid_lane_pairs, select_best_pair,
    infer_lane_pair_from_single_line, lane_offset_at_vehicle_cross_section,
)
from .line_detection import line_mask_coverage


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
            "lane": {"min_mm": 190.0, "max_mm": 210.0,
                     "target_mm": self.lane_width_mm, "height_mm": 0.0},
            "wall_outer": {"min_mm": 236.0, "max_mm": 241.0,
                           "target_mm": 236.0, "height_mm": 50.0},
        }
        if pair_profiles:
            for name, values in pair_profiles.items():
                if name in default_profiles and isinstance(values, dict):
                    default_profiles[name].update({
                        key: float(values[key])
                        for key in ("min_mm", "max_mm", "target_mm", "height_mm")
                        if key in values
                    })
        self.pair_profiles = default_profiles

        self.debug_pair_profile = str(debug_pair_profile or "auto").lower()
        if self.debug_pair_profile not in ("auto", *self.pair_profiles.keys()):
            self.debug_pair_profile = "auto"
        self.semantic_gate = bool(semantic_gate)
        self.semantic_min_coverage = float(semantic_min_coverage)
        self.semantic_line_thickness = int(semantic_line_thickness)

        self.projector = GroundIPMProjector(
            input_size=input_size,
            calibration_size=calibration_size,
            fx_px=fx_px, fy_px=fy_px, cx_px=cx_px, cy_px=cy_px,
            camera_height_mm=camera_height_mm, pitch_deg=pitch_deg,
            canvas_w=self.canvas_w, canvas_h_base=self.canvas_h_base,
            pixel_per_mm=self.pixel_per_mm, blind_spot_mm=self.blind_spot_mm,
            wall_height_mm=self.pair_profiles["wall_outer"].get("height_mm", 50.0),
        )
        self.selected_source_lines = []
        self.detected_source_lines = []
        self.selected_bev_segments = []
        self.candidate_bev_segments = []
        self.final_pair = None

        self.template_x_at_ref_mm: dict = {}
        self.identity_weights: dict = {}

    # ---------- 投影委托 ----------
    @property
    def matrix(self):
        return self.projector.matrix

    @matrix.setter
    def matrix(self, value):
        self.projector.matrix = np.asarray(value, dtype=np.float64)

    def _matrix_for_profile(self, profile_name):
        return self.projector.matrix_for_profile(profile_name)

    def _project_line(self, line, matrix=None):
        return self.projector.project_line(line, matrix)

    def _unproject_segment(self, segment, matrix=None):
        return self.projector.unproject_segment(segment, matrix)

    # ---------- 只读属性 ----------
    @property
    def canvas_size(self):
        return self.canvas_w, self.canvas_h

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
            "pid_error_mm": 0.0, "crossroad_detected": False,
            "distance_to_crossroad_mm": -1.0,
            "lane_angle_rad": 0.0, "quality_score": 0.0, "frame_dropped": True,
            "drop_reason": reason, "duty_cycle": 0.0,
            "pair_profile": None, "pair_target_mm": 0.0, "pair_distance_mm": 0.0,
            "semantic_boundary_coverage": 0.0,
            "vis_points": {"blind_spot_pts": [], "normal_lane_pts": [],
                           "crossroad_y": None,
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

        ground_candidates = [self._project_line(line, self._matrix_for_profile("lane"))
                             for line in source_lines]
        self.candidate_bev_segments = ground_candidates

        profile_names = ([self.debug_pair_profile] if self.debug_pair_profile != "auto"
                         else ["lane", "wall_outer"])
        chosen = None
        chosen_profile = None
        for profile_name in profile_names:
            profile = self.pair_profiles[profile_name]
            profile_candidates = [self._project_line(line, self._matrix_for_profile(profile_name))
                                  for line in source_lines]
            self.candidate_bev_segments = profile_candidates
            pairs = find_valid_lane_pairs(
                profile_candidates, self.pixel_per_mm, self.canvas_w,
                self.canvas_h, min_mm=profile["min_mm"], max_mm=profile["max_mm"],
                parallel_tol_deg=PARALLEL_ANGLE_TOL_DEG,
                vehicle_point=self.vehicle_footprint,
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
                        parallel_tol_deg=PARALLEL_ANGLE_TOL_DEG,
                        vehicle_point=self.vehicle_footprint,
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
            chosen = min(inferred_candidates,
                         key=lambda item: (abs(item["mid_x_mm"]),
                                           -source_lines[item["i"]].get("length", 0.0)))
            first_index = chosen["i"]
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
            first_segment = self.candidate_bev_segments[first_index]
            second_segment = self.candidate_bev_segments[second_index]
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
        self.final_pair = {
            **chosen,
            "left": left_segment, "right": right_segment,
            "left_img": left_img, "right_img": right_img,
            "left_inferred": left_inferred, "right_inferred": right_inferred,
            "inferred": left_inferred or right_inferred,
        }
        self.final_pair["pair_profile"] = chosen_profile or chosen.get("pair_profile", "lane")
        self.final_pair["target_mm"] = float(
            chosen.get("target_mm",
                       self.pair_profiles[self.final_pair["pair_profile"]]["target_mm"])
        )
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
            "lane_pair_mode": ("inferred_single_boundary"
                               if self.final_pair["inferred"] else "detected_pair"),
            "pair_profile": self.final_pair["pair_profile"],
            "pair_target_mm": self.final_pair["target_mm"],
            "pair_distance_mm": float(chosen["distance_mm"]),
            "semantic_boundary_coverage": self.final_pair["semantic_boundary_coverage"],
            "vis_points": {
                "blind_spot_pts": [],
                "normal_lane_pts": [tuple(point) for point in centre_points],
                "crossroad_y": None,
                "left_boundary_pts": [tuple(point) for point in left_segment],
                "right_boundary_pts": [tuple(point) for point in right_segment],
            },
        }
