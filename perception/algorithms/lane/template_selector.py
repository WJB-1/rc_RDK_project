# perception/algorithms/lane/template_selector.py
"""六线模板匹配 + 新地面系偏航/偏移计算。

这个 selector 涉及两个坐标系：
  1. ipm_camera    → BEV → 六线模板匹配 → 身份识别 → 车道横向
  2. ground_camera → 新地面系 → 偏航角 / 横向偏移量

两套相机参数完全独立，由 LaneConfig 分别注入。
新地面系的具体投影/拟合由 NewGroundProjector 承担。
"""
import math
import time
from bisect import bisect_left

import cv2
import numpy as np

from perception.algorithms.core.timing import block

from .config import CameraConfig
from .new_ground import NewGroundProjector
from .constants import (
    TEMPLATE_LINE_X_MM, TEMPLATE_LINE_ORDER, TEMPLATE_LINE_WEIGHTS,
    LANE_LINE_IDS, ALL_LINE_IDS, TEMPLATE_LINE_COLORS,
    GROUND_BODY_LENGTH_MM, GROUND_CAMERA_FORWARD_MM,
    GROUND_CAMERA_LATERAL_OFFSET_MM,
)
from .angle_template import template_positions_for_angle
from .ground_ipm_selector import GroundIPMLanePairSelector
from .line_geometry import (
    find_valid_lane_pairs, select_best_pair,
    infer_lane_pair_from_single_line, lane_offset_at_vehicle_cross_section,
)
from .line_detection import line_mask_coverage


class TemplateDistanceLaneSelector(GroundIPMLanePairSelector):
    """BPU-compatible six-line template matcher used by the real robot path.

    车道检测（BEV、模板匹配、身份识别）走 ipm_camera。
    偏航角和偏移量单独走 ground_camera，在"新地面系"里算。
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
                 ground_camera: CameraConfig = None,
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

        # ---- 新地面系投影器（独立于 ipm_camera，用 ground_camera） ----
        self.ground_camera = ground_camera
        self.new_ground = None
        if ground_camera is not None:
            self.new_ground = NewGroundProjector(
                camera=ground_camera,
                input_width=self.input_width,
                input_height=self.input_height,
            )

        self.template_identities = None
        self.template_match = None
        self.template_conf = None
        self._source_candidates = []
        self._last_new_ground_left_pts = []
        self._last_new_ground_right_pts = []
        self._last_new_ground_center_fit = None
        self._last_new_ground_source = "?"
        self._last_template_match_diagnostics = {}

    # ------------------------------------------------------------------
    # Ground reconstruction（旧 BEV 用，保留）
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
                distance = abs(
                    (lines[first]["params"]["theta"] - lines[second]["params"]["theta"] + math.pi / 2)
                    % math.pi - math.pi / 2
                )
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
        template = template_positions_for_angle(math.degrees(group_theta))
        return self._robot_center_inside_template_corridor(delta, group_theta, template)

    def _robot_center_inside_template_corridor(self, delta, group_theta, template):
        body_length = float(self.vehicle_geometry.get("body_length_mm", 142.0))
        camera_front = float(self.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        lateral = float(self.vehicle_geometry.get("camera_lateral_offset_mm", 0.0))
        y_center = -camera_front - body_length * 0.5
        tan_theta = math.tan(self._normalize_theta(group_theta))
        lane_x = {
            lid: template[lid] + delta + (y_center - self.y_ref_mm) * tan_theta
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
    # Template matching（旧 BEV 用，保留）
    # ------------------------------------------------------------------
    def _build_template_candidate(self, sorted_items, sorted_x, pairs, group_theta, template):
        assigned_ids = [TEMPLATE_LINE_ORDER[template_index] for _, template_index in pairs]
        if len(assigned_ids) < 3:
            return None
        if self.require_both_lane_lines and not all(line_id in assigned_ids for line_id in LANE_LINE_IDS):
            return None

        weights = [float(TEMPLATE_LINE_WEIGHTS[line_id]) for line_id in assigned_ids]
        total_weight = sum(weights)
        if total_weight <= 0.0:
            return None
        delta = sum(
            weights[index] * (sorted_x[line_index] - template[line_id])
            for index, ((line_index, _), line_id) in enumerate(zip(pairs, assigned_ids))
        ) / total_weight
        fit_loss = sum(
            weights[index] * (sorted_x[line_index] - template[line_id] - delta) ** 2
            for index, ((line_index, _), line_id) in enumerate(zip(pairs, assigned_ids))
        ) / total_weight
        rms = math.sqrt(fit_loss)
        robot_inside, pose = self._robot_center_inside_template_corridor(delta, group_theta, template)
        if not robot_inside or rms > self.max_match_rms_mm:
            return None

        coverage_reward = total_weight
        return {
            "assignment": {
                line_id: sorted_items[line_index]
                for (line_index, _), line_id in zip(pairs, assigned_ids)
            },
            "assigned_ids": assigned_ids,
            "delta": float(delta),
            "loss": float(fit_loss - self.line_count_reward_lambda * coverage_reward),
            "fit_loss": float(fit_loss),
            "coverage_reward": float(coverage_reward),
            "rms": float(rms),
            "group_theta": float(group_theta),
            "n_matched": len(pairs),
            "robot_center_inside": True,
            "pose": pose,
        }

    @staticmethod
    def _candidate_deltas(sorted_x, template):
        return sorted({
            round(float(x_ref - template[line_id]), 6)
            for x_ref in sorted_x
            for line_id in TEMPLATE_LINE_ORDER
        })

    def _template_index_subsets(self):
        lane_indices = {
            index for index, line_id in enumerate(TEMPLATE_LINE_ORDER)
            if line_id in LANE_LINE_IDS
        }
        subsets = []
        for mask in range(1 << len(TEMPLATE_LINE_ORDER)):
            indices = tuple(
                index for index in range(len(TEMPLATE_LINE_ORDER))
                if mask & (1 << index)
            )
            if len(indices) < 3:
                continue
            if self.require_both_lane_lines and not lane_indices.issubset(indices):
                continue
            subsets.append(indices)
        return subsets

    @staticmethod
    def _best_ordered_pairs_for_delta(sorted_x, template, template_indices, delta):
        if len(template_indices) > len(sorted_x):
            return None
        previous_costs = None
        parent_rows = []
        for template_index in template_indices:
            line_id = TEMPLATE_LINE_ORDER[template_index]
            weight = float(TEMPLATE_LINE_WEIGHTS[line_id])
            costs = [math.inf] * len(sorted_x)
            parents = [-1] * len(sorted_x)
            best_previous_cost = math.inf
            best_previous_index = -1
            for line_index, x_at_ref in enumerate(sorted_x):
                residual = x_at_ref - template[line_id] - delta
                point_cost = weight * residual * residual
                if previous_costs is None:
                    costs[line_index] = point_cost
                    continue
                if line_index:
                    candidate_cost = previous_costs[line_index - 1]
                    if candidate_cost < best_previous_cost:
                        best_previous_cost = candidate_cost
                        best_previous_index = line_index - 1
                if best_previous_index >= 0:
                    costs[line_index] = best_previous_cost + point_cost
                    parents[line_index] = best_previous_index
            previous_costs = costs
            parent_rows.append(parents)

        final_index = min(range(len(sorted_x)), key=previous_costs.__getitem__)
        if not math.isfinite(previous_costs[final_index]):
            return None
        pairs = []
        for row_index in range(len(template_indices) - 1, -1, -1):
            pairs.append((final_index, template_indices[row_index]))
            final_index = parent_rows[row_index][final_index]
        return tuple(reversed(pairs))

    @staticmethod
    def _nearest_sorted_line_index(negated_x, target_x, sorted_x):
        insertion_index = bisect_left(negated_x, -target_x)
        candidate_indices = []
        if insertion_index < len(sorted_x):
            candidate_indices.append(insertion_index)
        if insertion_index:
            candidate_indices.append(insertion_index - 1)
        return min(candidate_indices, key=lambda index: abs(sorted_x[index] - target_x))

    def _match_group_to_template_fast(self, group):
        if len(group) < 3:
            return None

        enriched = [
            {"item": item, "x_at_ref": self._line_x_at_ref(item["params"], self.y_ref_mm)}
            for item in group
        ]
        enriched.sort(key=lambda entry: entry["x_at_ref"], reverse=True)
        sorted_x = [entry["x_at_ref"] for entry in enriched]
        sorted_items = [entry["item"] for entry in enriched]
        negated_x = [-x_at_ref for x_at_ref in sorted_x]
        group_theta = self._estimate_group_theta(group)
        template = template_positions_for_angle(math.degrees(group_theta))
        max_point_residual = self.max_match_rms_mm * 2.0
        best = None
        second_loss = None
        attempts = 0
        accepted = 0

        for anchor_x in sorted_x:
            for anchor_line_id in TEMPLATE_LINE_ORDER:
                attempts += 1
                delta = anchor_x - template[anchor_line_id]
                pairs_by_line = {}
                for template_index, line_id in enumerate(TEMPLATE_LINE_ORDER):
                    target_x = template[line_id] + delta
                    line_index = self._nearest_sorted_line_index(negated_x, target_x, sorted_x)
                    residual = abs(sorted_x[line_index] - target_x)
                    if residual > max_point_residual:
                        continue
                    previous = pairs_by_line.get(line_index)
                    if previous is None or residual < previous[0]:
                        pairs_by_line[line_index] = (residual, template_index)

                pairs = tuple(
                    (line_index, template_index)
                    for line_index, (_, template_index) in sorted(pairs_by_line.items())
                )
                candidate = self._build_template_candidate(
                    sorted_items, sorted_x, pairs, group_theta, template,
                )
                if candidate is None:
                    continue
                accepted += 1
                candidate["match_strategy"] = "ordered_anchor"
                if best is None or candidate["loss"] < best["loss"]:
                    if best is not None:
                        second_loss = best["loss"]
                    best = candidate
                elif second_loss is None or candidate["loss"] < second_loss:
                    second_loss = candidate["loss"]

        self._current_template_group_diagnostics = {
            "evaluated_combinations": attempts,
            "accepted_combinations": accepted,
            "direct_six_attempted": False,
            "fast_anchor_attempts": attempts,
            "dp_delta_candidates": 0,
            "dp_state_updates": 0,
        }
        if best is None:
            return None
        loss_gap = math.inf if second_loss is None else second_loss - best["loss"]
        best["second_best_loss"] = second_loss
        best["loss_gap"] = loss_gap
        best["near_tie"] = loss_gap <= getattr(self, "match_tie_loss_epsilon", 1e-6)
        return best

    def _match_group_to_template_ordered(self, group):
        if len(group) < 3:
            return None

        enriched = [
            {"item": item, "x_at_ref": self._line_x_at_ref(item["params"], self.y_ref_mm)}
            for item in group
        ]
        enriched.sort(key=lambda entry: entry["x_at_ref"], reverse=True)
        sorted_x = [entry["x_at_ref"] for entry in enriched]
        sorted_items = [entry["item"] for entry in enriched]
        group_theta = self._estimate_group_theta(group)
        template = template_positions_for_angle(math.degrees(group_theta))
        template_count = len(TEMPLATE_LINE_ORDER)
        line_count = len(sorted_items)

        diagnostics = {
            "evaluated_combinations": 0,
            "accepted_combinations": 0,
            "direct_six_attempted": False,
            "dp_delta_candidates": 0,
            "dp_state_updates": 0,
        }

        if line_count == template_count:
            diagnostics["direct_six_attempted"] = True
            direct = self._build_template_candidate(
                sorted_items, sorted_x,
                [(index, index) for index in range(template_count)],
                group_theta, template,
            )
            diagnostics["evaluated_combinations"] += 1
            if direct is not None:
                diagnostics["accepted_combinations"] += 1
                direct["match_strategy"] = "ordered_six"
                direct["second_best_loss"] = None
                direct["loss_gap"] = math.inf
                direct["near_tie"] = False
                self._current_template_group_diagnostics = diagnostics
                return direct

        fast = self._match_group_to_template_fast(group)
        if fast is not None:
            return fast

        best = None
        second_loss = None
        template_subsets = self._template_index_subsets()
        for delta in self._candidate_deltas(sorted_x, template):
            diagnostics["dp_delta_candidates"] += 1
            for template_indices in template_subsets:
                pairs = self._best_ordered_pairs_for_delta(
                    sorted_x, template, template_indices, delta,
                )
                diagnostics["dp_state_updates"] += line_count * len(template_indices)
                if pairs is None:
                    continue
                diagnostics["evaluated_combinations"] += 1
                candidate = self._build_template_candidate(
                    sorted_items, sorted_x, pairs, group_theta, template,
                )
                if candidate is None:
                    continue
                diagnostics["accepted_combinations"] += 1
                candidate["match_strategy"] = "ordered_dp"
                if best is None or candidate["loss"] < best["loss"]:
                    if best is not None:
                        second_loss = best["loss"]
                    best = candidate
                elif second_loss is None or candidate["loss"] < second_loss:
                    second_loss = candidate["loss"]

        if best is not None:
            loss_gap = math.inf if second_loss is None else second_loss - best["loss"]
            best["second_best_loss"] = second_loss
            best["loss_gap"] = loss_gap
            best["near_tie"] = loss_gap <= self.match_tie_loss_epsilon
        self._current_template_group_diagnostics = diagnostics
        return best

    def _match_group_to_template(self, group):
        return self._match_group_to_template_ordered(group)

    def _match_all_groups(self, ground_items):
        started_at = time.perf_counter()
        groups = self._parallel_groups(ground_items)
        max_theta = math.radians(self.max_forward_line_angle_deg)
        valid_groups = [
            group for group in groups
            if 3 <= len(group) <= 10
            and all(abs(self._normalize_theta(ground_items[index]["params"]["theta"])) <= max_theta
                    for index in group)
        ]
        best = None
        evaluated_combinations = 0
        accepted_combinations = 0
        direct_six_attempted = False
        for group in valid_groups:
            candidate = self._match_group_to_template([ground_items[index] for index in group])
            group_diagnostics = self._current_template_group_diagnostics
            evaluated_combinations += group_diagnostics["evaluated_combinations"]
            accepted_combinations += group_diagnostics["accepted_combinations"]
            direct_six_attempted |= group_diagnostics["direct_six_attempted"]
            if candidate is not None and (best is None or candidate["loss"] < best["loss"]):
                best = candidate
        self._record_template_match_diagnostics(
            input_line_count=len(ground_items),
            group_sizes=[len(group) for group in groups],
            valid_group_sizes=[len(group) for group in valid_groups],
            evaluated_combinations=evaluated_combinations,
            accepted_combinations=accepted_combinations,
            direct_six_attempted=direct_six_attempted,
            elapsed_ms=(time.perf_counter() - started_at) * 1000.0,
        )
        return best, len(groups)

    def _record_template_match_diagnostics(self, input_line_count, group_sizes, valid_group_sizes,
                                           evaluated_combinations, accepted_combinations,
                                           direct_six_attempted, elapsed_ms):
        self._last_template_match_diagnostics = {
            "input_line_count": int(input_line_count),
            "parallel_group_sizes": list(group_sizes),
            "valid_group_sizes": list(valid_group_sizes),
            "evaluated_combinations": int(evaluated_combinations),
            "accepted_combinations": int(accepted_combinations),
            "direct_six_attempted": bool(direct_six_attempted),
            "elapsed_ms": float(elapsed_ms),
            "avg_combination_us": (
                float(elapsed_ms) * 1000.0 / evaluated_combinations
                if evaluated_combinations else 0.0
            ),
        }

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
        template = template_positions_for_angle(math.degrees(inferred_theta))
        result = {}
        for lid in ALL_LINE_IDS:
            template_x_ref = template[lid] + delta
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
    # 新地面系：偏航角 / 偏移量计算
    # ------------------------------------------------------------------
    def _compute_ground_yaw_offset(self, left_bev, right_bev, left_source=None, right_source=None):
        """将已保留的原图线段直接投影至 ground_camera 地面系。"""
        if self.new_ground is None:
            return None

        lane_matrix = self._matrix_for_profile("lane")
        inv_lane_matrix = np.linalg.inv(lane_matrix)

        def _to_ground_pts(bev_segment, source_line):
            if bev_segment is None:
                return []
            if source_line is not None:
                with block("lane.image_to_new_ground"):
                    return self.new_ground.source_line_to_ground(source_line)
            # 模板匹配后的 BEV 线段，使用 lane-IPM 的逆变换还原至图像。
            with block("lane.bev_to_image"):
                bev_f = np.asarray(bev_segment, dtype=np.float32).reshape(1, 2, 2)
                img_back = cv2.perspectiveTransform(bev_f, inv_lane_matrix)[0]
            back_line = {
                "x1": float(img_back[0][0]), "y1": float(img_back[0][1]),
                "x2": float(img_back[1][0]), "y2": float(img_back[1][1]),
            }
            # 3) 图像 → 地面（ground_camera 外参）
            with block("lane.image_to_new_ground"):
                return self.new_ground.source_line_to_ground(back_line)

        with block("lane.source_to_new_ground"):
            left_ground_pts = _to_ground_pts(left_bev, left_source)
            right_ground_pts = _to_ground_pts(right_bev, right_source)

        with block("lane.fit_new_ground"):
            left_fit = self.new_ground.fit_centerline(left_ground_pts)
            right_fit = self.new_ground.fit_centerline(right_ground_pts)

        self._last_new_ground_left_pts = list(left_ground_pts)
        self._last_new_ground_right_pts = list(right_ground_pts)

        if left_fit is not None and right_fit is not None:
            a_c = 0.5 * (left_fit[0] + right_fit[0])
            b_c = 0.5 * (left_fit[1] + right_fit[1])
            theta_source = "new_ground_pair"
        elif left_fit is not None:
            a_c, b_c = left_fit
            theta_source = "new_ground_left_only"
        elif right_fit is not None:
            a_c, b_c = right_fit
            theta_source = "new_ground_right_only"
        else:
            self._last_new_ground_center_fit = None
            self._last_new_ground_source = "new_ground_empty"
            return None

        self._last_new_ground_center_fit = (a_c, b_c)
        self._last_new_ground_source = theta_source

        theta_ground = math.atan(a_c)
        body_length = float(self.vehicle_geometry.get(
            "body_length_mm", GROUND_BODY_LENGTH_MM))
        camera_forward = float(self.vehicle_geometry.get(
            "camera_forward_of_body_front_mm", GROUND_CAMERA_FORWARD_MM))
        camera_lateral_offset = float(self.vehicle_geometry.get(
            "camera_lateral_offset_mm", GROUND_CAMERA_LATERAL_OFFSET_MM))
        y_vehicle_center_mm = -camera_forward - body_length * 0.5
        offset_mm = a_c * y_vehicle_center_mm + b_c - camera_lateral_offset

        with block("lane.new_ground_to_image"):
            ys_plot = np.linspace(200.0, 3000.0, 20)
            pts_img = []
            for y in ys_plot:
                x = a_c * y + b_c
                p = self.new_ground.ground_to_image(x, y)
                if p is not None:
                    pts_img.append(p)
            if len(pts_img) >= 2:
                center_img_p1 = pts_img[0]
                center_img_p2 = pts_img[-1]
            else:
                center_img_p1 = None
                center_img_p2 = None

        return offset_mm, theta_ground, theta_source, center_img_p1, center_img_p2

    # ------------------------------------------------------------------
    # Frame analysis
    # ------------------------------------------------------------------
    def analyze(self, source_lines, semantic_mask=None):
        self.selected_source_lines, self.detected_source_lines = [], []
        self.selected_bev_segments, self.final_pair = [], None
        self._source_candidates = list(source_lines)
        if self.semantic_gate and semantic_mask is None:
            state = self._empty_state("semantic_mask_required")
            state.update({"pair_profile": "template_distance", "template_match_count": 0})
            return state

        # ---------------- 车道检测：BEV 全流程（ipm_camera） ----------------
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
            parallel_angle = abs(
                (left_info["theta"] - right_info["theta"] + math.pi / 2) % math.pi - math.pi / 2
            )
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
        # 偏航角和偏移量：新地面系（ground_camera），失败则 BEV fallback
        # ============================================================
        ground_result = self._compute_ground_yaw_offset(
            left_segment,
            right_segment,
            left_info["source_line"],
            right_info["source_line"],
        )
        if ground_result is not None:
            offset_mm, lane_angle_rad, theta_source, center_img_p1, center_img_p2 = ground_result
            if center_img_p1 is None or center_img_p2 is None:
                center_img_p1 = (self.input_width / 2.0, self.input_height - 1)
                center_img_p2 = (self.input_width / 2.0, 0.0)
        else:
            # BEV fallback：用 BEV 中心线方向兜底
            centre_direction = (self._forward_direction(left_segment)
                                + self._forward_direction(right_segment))
            lane_angle_rad = math.atan2(
                float(centre_direction[0]), float(-centre_direction[1]))
            theta_source = ("bev_fallback_no_ground_camera"
                            if self.new_ground is None
                            else "bev_fallback_new_ground_empty")
            offset_mm = 0.0
            center_img_p1 = (self.input_width / 2.0, self.input_height - 1)
            center_img_p2 = (self.input_width / 2.0, 0.0)

        self.final_pair["center_image_p1"] = center_img_p1
        self.final_pair["center_image_p2"] = center_img_p2
        self.final_pair["theta_source"] = theta_source
        self.final_pair["lane_angle_deg"] = math.degrees(lane_angle_rad)
        self.final_pair["offset_mm"] = float(offset_mm)

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
    
        # ---------------- 可视化委托 ----------------
    def draw_bev_view(self):
        from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer
        return LaneSelectorRenderer(self).draw_bev_view()

    def draw_original_view(self, processing_frame):
        from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer
        return LaneSelectorRenderer(self).draw_original_view(processing_frame)

    def render_new_ground_bev(self):
        from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer
        return LaneSelectorRenderer(self).render_new_ground_bev()

    def render_original_with_ground_centerline(self, processing_frame):
        from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer
        return LaneSelectorRenderer(self).render_original_with_ground_centerline(processing_frame)
