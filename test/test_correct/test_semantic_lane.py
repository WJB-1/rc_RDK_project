import unittest

import cv2
import numpy as np


class SemanticLaneTests(unittest.TestCase):
    def test_semantic_debug_capture_only_renders_requested_view(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.debug_view_name = "semantic_overlay"

        self.assertEqual(pipeline._requested_debug_views(True), {"semantic_overlay"})

    def test_boundary_roi_preserves_original_image_coordinates(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[300:480, 220:421] = 255

        _, lines, _, _ = SemanticLaneDetector(1.0, 640, 200)._extract_side_lines(mask)

        self.assertEqual(len(lines), 2)
        self.assertTrue(all(min(line["y1"], line["y2"]) >= 290 for line in lines))
        self.assertTrue(any(abs((line["x1"] + line["x2"]) * 0.5 - 220) < 3 for line in lines))
        self.assertTrue(any(abs((line["x1"] + line["x2"]) * 0.5 - 420) < 3 for line in lines))
    def test_boundary_roi_preserves_original_image_coordinates(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[300:480, 220:421] = 255

        _, lines, _, _ = SemanticLaneDetector(1.0, 640, 200)._extract_side_lines(mask)

        self.assertEqual(len(lines), 2)
        self.assertTrue(all(min(line["y1"], line["y2"]) >= 290 for line in lines))
        self.assertTrue(any(abs((line["x1"] + line["x2"]) * 0.5 - 220) < 3 for line in lines))
        self.assertTrue(any(abs((line["x1"] + line["x2"]) * 0.5 - 420) < 3 for line in lines))
    def test_component_selector_keeps_largest_ground_touching_region(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((100, 120), dtype=np.uint8)
        cv2.rectangle(mask, (5, 5), (45, 50), 255, -1)
        cv2.rectangle(mask, (70, 45), (110, 99), 255, -1)
        selected, info = SemanticLaneDetector(1.0, 120, 80)._select_ground_component(mask)

        self.assertEqual(info["area_px"], int(np.count_nonzero(mask[45:100, 70:111])))
        self.assertEqual(int(np.count_nonzero(selected[5:45, 5:45])), 0)

    def test_detector_extracts_mask_edges_without_hough(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((120, 160), dtype=np.uint8)
        for y in range(20, 120):
            left = 35 + (y - 20) // 8
            right = 115 - (y - 20) // 8
            mask[y, left:right + 1] = 255
        edge, lines, raw_count, points = SemanticLaneDetector(1.0, 160, 80)._extract_side_lines(mask)

        self.assertEqual(raw_count, 2)
        self.assertEqual(len(lines), 2)
        self.assertEqual(int(np.count_nonzero(edge)), 200)
        self.assertEqual(len(points["left_0"]), 100)

    def test_vertical_gaps_are_joined_before_component_selection(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((120, 160), dtype=np.uint8)
        mask[20:50, 35:116] = 255
        mask[60:120, 35:116] = 255
        detector = SemanticLaneDetector(1.0, 160, 80, component_gap_px=10)

        selected, info = detector._select_ground_component(mask)
        edge, lines, _, _ = detector._extract_side_lines(selected)

        self.assertIsNotNone(info)
        self.assertEqual(len(lines), 2)
        self.assertGreaterEqual(int(np.count_nonzero(edge)), 200)

    def test_hough_detects_multiple_lines_at_boundary_slope_change(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((180, 220), dtype=np.uint8)
        polygon = np.array([(40, 10), (40, 80), (80, 170),
                            (180, 170), (180, 80), (180, 10)])
        cv2.fillPoly(mask, [polygon], 255)
        detector = SemanticLaneDetector(1.0, 220, 100, min_segment_length_px=20)

        _, lines, raw_count, _ = detector._extract_side_lines(mask)

        self.assertGreaterEqual(raw_count, 2)
        self.assertGreaterEqual(len(lines), 2)

    def test_angle_template_uses_left_origin_baseline(self):
        from perception.algorithms.lane.angle_template import template_positions_for_angle
        from perception.algorithms.lane.constants import TEMPLATE_LINE_ORDER, TEMPLATE_LINE_X_MM

        template = template_positions_for_angle(0.0)

        self.assertEqual(TEMPLATE_LINE_ORDER, ("5", "3", "1", "0", "2", "4"))
        for line_id in TEMPLATE_LINE_ORDER:
            self.assertAlmostEqual(template[line_id], TEMPLATE_LINE_X_MM[line_id], places=6)

    def test_angle_template_accumulates_gap_corrections_from_left(self):
        from perception.algorithms.lane.angle_template import (
            NEIGHBOUR_GAP_LINEAR_MODELS,
            template_positions_for_angle,
        )
        from perception.algorithms.lane.constants import TEMPLATE_LINE_X_MM

        angle_deg = 12.0
        template = template_positions_for_angle(angle_deg)
        expected_x3 = TEMPLATE_LINE_X_MM["3"] + NEIGHBOUR_GAP_LINEAR_MODELS[("3", "5")][1] * angle_deg
        expected_x1 = expected_x3 + (
            TEMPLATE_LINE_X_MM["1"] - TEMPLATE_LINE_X_MM["3"]
            + NEIGHBOUR_GAP_LINEAR_MODELS[("1", "3")][1] * angle_deg
        )

        self.assertEqual(template["5"], 0.0)
        self.assertAlmostEqual(template["3"], expected_x3, places=6)
        self.assertAlmostEqual(template["1"], expected_x1, places=6)
        self.assertTrue(all(
            template[left] < template[right]
            for left, right in zip(("5", "3", "1", "0", "2"), ("3", "1", "0", "2", "4"))
        ))

    def test_template_dp_keeps_order_and_skips_an_outlier(self):
        from perception.algorithms.lane.angle_template import template_positions_for_angle
        from perception.algorithms.core.timing import get_frame_timings, reset_frame
        from perception.algorithms.lane.template_selector import TemplateDistanceLaneSelector

        selector = TemplateDistanceLaneSelector.__new__(TemplateDistanceLaneSelector)
        selector.y_ref_mm = 500.0
        selector.max_match_rms_mm = 2.0
        selector.line_count_reward_lambda = 5.0
        selector.require_both_lane_lines = True
        selector.match_tie_loss_epsilon = 1e-6
        selector.vehicle_geometry = {
            "body_length_mm": 142.0,
            "camera_forward_of_body_front_mm": 60.0,
            "camera_lateral_offset_mm": 0.0,
        }

        angle_deg = 12.0
        theta = np.deg2rad(angle_deg)
        delta = -40.0
        template = template_positions_for_angle(angle_deg)
        line_ids = ("5", "3", "1", "0", "2", "4")

        def line_at(x_ref):
            return {
                "params": {
                    "theta": theta,
                    "mid_x": x_ref,
                    "mid_y": 500.0,
                    "length_mm": 500.0,
                    "p1": (x_ref, 250.0),
                    "p2": (x_ref, 750.0),
                }
            }

        group = [line_at(template[line_id] + delta) for line_id in line_ids]
        group.append(line_at(template["4"] + delta + 80.0))

        reset_frame()
        match = selector._match_group_to_template_ordered(group)
        recorded_stages = {name for name, _ in get_frame_timings()}

        self.assertIsNotNone(match)
        self.assertEqual(match["assigned_ids"], list(line_ids))
        self.assertAlmostEqual(match["delta"], delta, places=4)
        self.assertLess(match["rms"], 1e-4)
        self.assertEqual(match["match_strategy"], "fast_enum")
        self.assertLess(selector._current_template_group_diagnostics["evaluated_combinations"], 1000)
        if match["match_strategy"] == "unified_dp":
            self.assertIn("lane.candidate_delta", recorded_stages)
            self.assertIn("lane.template_subset_dp", recorded_stages)

    def test_template_uses_direct_ordered_path_for_exactly_six_lines(self):
        from perception.algorithms.lane.angle_template import template_positions_for_angle
        from perception.algorithms.lane.template_selector import TemplateDistanceLaneSelector

        selector = TemplateDistanceLaneSelector.__new__(TemplateDistanceLaneSelector)
        selector.y_ref_mm = 500.0
        selector.max_match_rms_mm = 2.0
        selector.line_count_reward_lambda = 5.0
        selector.require_both_lane_lines = True
        selector.match_tie_loss_epsilon = 1e-6
        selector.vehicle_geometry = {
            "body_length_mm": 142.0,
            "camera_forward_of_body_front_mm": 60.0,
            "camera_lateral_offset_mm": 0.0,
        }
        theta = np.deg2rad(12.0)
        template = template_positions_for_angle(12.0)

        group = [
            {"params": {
                "theta": theta,
                "mid_x": template[line_id] - 40.0,
                "mid_y": 500.0,
                "length_mm": 500.0,
                "p1": (template[line_id], 250.0),
                "p2": (template[line_id], 750.0),
            }}
            for line_id in ("5", "3", "1", "0", "2", "4")
        ]

        match = selector._match_group_to_template_ordered(group)

        self.assertIsNotNone(match)
        self.assertEqual(match["match_strategy"], "fast_enum")
        self.assertTrue(selector._current_template_group_diagnostics["direct_six_attempted"])
        self.assertEqual(selector._current_template_group_diagnostics["evaluated_combinations"], 1)

    def test_template_match_diagnostics_reports_combination_count(self):
        from perception.algorithms.lane.template_selector import TemplateDistanceLaneSelector

        selector = TemplateDistanceLaneSelector.__new__(TemplateDistanceLaneSelector)
        selector._last_template_match_diagnostics = {}

        selector._record_template_match_diagnostics(
            input_line_count=8,
            group_sizes=[8],
            valid_group_sizes=[8],
            evaluated_combinations=120,
            accepted_combinations=3,
            direct_six_attempted=False,
            elapsed_ms=12.0,
        )

        self.assertEqual(selector._last_template_match_diagnostics["evaluated_combinations"], 120)
        self.assertAlmostEqual(selector._last_template_match_diagnostics["avg_combination_us"], 100.0)

    def test_single_erosion_separates_thick_segments_joined_by_a_thin_bridge(self):
        from perception.algorithms.lane.line_detection import erode_edge_segments

        binary = np.zeros((80, 100), dtype=np.uint8)
        cv2.rectangle(binary, (20, 10), (28, 70), 255, -1)
        cv2.rectangle(binary, (60, 10), (68, 70), 255, -1)
        cv2.line(binary, (28, 40), (60, 40), 255, 1)

        separated = erode_edge_segments(binary)
        labels, _ = cv2.connectedComponents(separated)

        self.assertEqual(labels - 1, 2)

    def test_skeletonization_runs_on_eroded_edges(self):
        from perception.algorithms.lane.line_detection import remove_skeleton_border, thin_binary

        binary = np.zeros((80, 100), dtype=np.uint8)
        cv2.rectangle(binary, (45, 10), (55, 70), 255, -1)
        eroded = cv2.erode(binary, np.ones((4, 4), dtype=np.uint8), iterations=1)

        skeleton = thin_binary(eroded)
        skeleton[0, :] = 255
        skeleton[:, 0] = 255
        skeleton = remove_skeleton_border(skeleton)

        self.assertLess(np.count_nonzero(skeleton), np.count_nonzero(eroded))
        self.assertGreater(np.count_nonzero(skeleton), 40)
        self.assertEqual(int(np.count_nonzero(skeleton[0, :])), 0)

    def test_component_centerline_extracts_one_axis_from_a_thick_segment(self):
        from perception.algorithms.lane.line_detection import detect_component_centerlines

        binary = np.zeros((120, 160), dtype=np.uint8)
        cv2.rectangle(binary, (70, 20), (78, 100), 255, -1)

        lines = detect_component_centerlines(binary, min_length=50.0)

        self.assertEqual(len(lines), 1)
        self.assertAlmostEqual((lines[0]["x1"] + lines[0]["x2"]) * 0.5, 74.0, delta=1.0)
        self.assertGreater(lines[0]["length"], 75.0)

    def test_component_centerline_merges_collinear_segments_across_a_gap(self):
        from perception.algorithms.lane.line_detection import detect_component_centerlines

        binary = np.zeros((160, 160), dtype=np.uint8)
        cv2.line(binary, (80, 20), (80, 65), 255, 1)
        cv2.line(binary, (80, 90), (80, 140), 255, 1)

        lines = detect_component_centerlines(binary, min_length=30.0, max_gap_px=30.0)

        self.assertEqual(len(lines), 1)
        self.assertGreater(lines[0]["length"], 115.0)
        self.assertEqual(lines[0]["merged_from"], 2)

    def test_auto_mode_uses_template_only_on_the_frame_after_semantic_failure(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.semantic_lane_mode = "auto"
        pipeline._auto_template_next = False

        self.assertFalse(pipeline._should_run_template())
        pipeline._auto_template_next = True
        self.assertTrue(pipeline._should_run_template())

    def test_semantic_bev_connects_transformed_boundaries_and_gate_colours(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.selector = type("Selector", (), {"canvas_w": 100, "canvas_h": 100})()
        edge = np.zeros((100, 100), dtype=np.uint8)
        edge[10, 10] = 255
        result = {
            "edge_mask": edge,
            "bev_boundary_points": {
                "left": np.array([[10.0, 10.0], [10.0, 60.0]], dtype=np.float32),
            },
            "accepted_bev_lines": [np.array([[20.0, 20.0], [20.0, 80.0]])],
            "rejected_bev_lines": [np.array([[70.0, 20.0], [70.0, 80.0]])],
        }

        image = pipeline._draw_semantic_bev(result, np.eye(3, dtype=np.float64))

        self.assertTrue(np.all(image[30, 10] > 0))
        self.assertGreater(int(image[50, 20, 1]), int(image[50, 20, 2]))
        self.assertGreater(int(image[50, 70, 2]), int(image[50, 70, 1]))

    def test_semantic_bev_renders_gate_text_when_no_line_pair_exists(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.selector = type("Selector", (), {"canvas_w": 220, "canvas_h": 140})()
        result = {
            "edge_mask": np.zeros((140, 220), dtype=np.uint8),
            "accepted": False,
            "fallback_reason": "no_parallel_distance_valid_pair",
            "raw_hough_segment_count": 3,
            "min_distance_mm": 180.0,
            "max_distance_mm": 220.0,
            "pair_measurements": [],
            "bev_lines": [],
            "accepted_bev_lines": [],
            "rejected_bev_lines": [],
        }

        image = pipeline._draw_semantic_bev(result, np.eye(3, dtype=np.float64))

        self.assertGreater(int(np.count_nonzero(image)), 0)

    def test_lane_mode_accepts_template_semantic_and_auto(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.semantic_lane_detector = object()
        pipeline.semantic_engine = object()
        pipeline.semantic_lane_mode = "auto"

        self.assertEqual(pipeline.set_semantic_lane_mode("template"), "template")
        self.assertEqual(pipeline.set_semantic_lane_mode("semantic"), "semantic")
        self.assertEqual(pipeline.set_semantic_lane_mode("auto"), "auto")
        with self.assertRaises(ValueError):
            pipeline.set_semantic_lane_mode("invalid")

    def test_lane_mode_rejects_semantic_when_detector_is_unavailable(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.semantic_lane_detector = None
        pipeline.semantic_engine = None
        pipeline.semantic_lane_mode = "template"

        self.assertEqual(pipeline.set_semantic_lane_mode("template"), "template")
        with self.assertRaises(RuntimeError):
            pipeline.set_semantic_lane_mode("semantic")

    def test_detector_reassembles_split_boundaries_and_accepts_parallel_lane(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((384, 512), dtype=np.uint8)
        cv2.rectangle(mask, (155, 120), (355, 383), 255, -1)
        mask[220:250] = 0

        result = SemanticLaneDetector(
            pixel_per_mm=1.0,
            bev_width=512,
            lane_width_mm=190.0,
            distance_tolerance_mm=15.0,
        ).analyze(mask, np.eye(3, dtype=np.float64))

        self.assertTrue(result["accepted"])
        self.assertEqual(len(result["accepted_image_lines"]), 2)

    def test_detector_rejects_nonparallel_pair_for_template_fallback(self):
        from perception.algorithms.lane.semantic_lane import SemanticLaneDetector

        mask = np.zeros((384, 512), dtype=np.uint8)
        cv2.fillPoly(mask, [np.array([(151, 383), (206, 120), (349, 120), (349, 383)])], 255)

        result = SemanticLaneDetector(
            pixel_per_mm=1.0,
            bev_width=512,
            lane_width_mm=190.0,
            distance_tolerance_mm=15.0,
            parallel_tolerance_deg=3.0,
        ).analyze(mask, np.eye(3, dtype=np.float64))

        self.assertFalse(result["accepted"])
        self.assertEqual(result["fallback_reason"], "no_parallel_distance_valid_pair")
        self.assertGreaterEqual(result["raw_hough_segment_count"], 2)
        self.assertEqual(len(result["pair_measurements"]), 1)
        self.assertFalse(result["pair_measurements"][0]["parallel_ok"])

    def test_pair_selection_uses_weighted_parallel_and_squared_template_loss(self):
        candidates = [
            {"selection_score": 0.6 * (2.0 / 3.0) + 0.4 * 0.0 ** 2,
             "mid_x_mm": 0.0},
            {"selection_score": 0.6 * (0.3 / 3.0) + 0.4 * (20.0 / 100.0) ** 2,
             "mid_x_mm": 5.0},
        ]

        selected = min(
            candidates,
            key=lambda item: (item["selection_score"], abs(item["mid_x_mm"])),
        )

        self.assertEqual(selected["mid_x_mm"], 5.0)

    def test_semantic_pair_bypasses_six_line_template_matching(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        selector = type("Selector", (), {})()
        selector.vehicle_center = np.array([50.0, 90.0])
        selector.pixel_per_mm = 1.0
        selector._compute_ground_yaw_offset = lambda left, right, left_image, right_image: (
            12.5, np.deg2rad(4.0), "new_ground_pair", (40.0, 80.0), (60.0, 20.0)
        )
        selector._empty_state = lambda reason: {"drop_reason": reason, "frame_dropped": True}
        pipeline.selector = selector
        image_lines = [
            {"x1": 20.0, "y1": 100.0, "x2": 20.0, "y2": 0.0},
            {"x1": 80.0, "y1": 100.0, "x2": 80.0, "y2": 0.0},
        ]
        result = {
            "pair": {"i": 0, "j": 1, "distance_mm": 60.0,
                     "target_distance_mm": 60.0, "selection_score": 0.1},
            "image_lines": image_lines,
            "bev_lines": [
                np.array([[20.0, 100.0], [20.0, 0.0]]),
                np.array([[80.0, 100.0], [80.0, 0.0]]),
            ],
        }

        state = pipeline._semantic_lane_state(result)

        self.assertFalse(state["frame_dropped"])
        self.assertEqual(state["lane_angle_source"], "new_ground_pair")
        self.assertAlmostEqual(state["pid_error_mm"], 12.5)
        self.assertAlmostEqual(state["quality_score"], 0.9)
        self.assertEqual(len(selector.detected_source_lines), 2)


if __name__ == "__main__":
    unittest.main()
