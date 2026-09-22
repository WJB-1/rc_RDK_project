import unittest

import cv2
import numpy as np


class SemanticLaneTests(unittest.TestCase):
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

    def test_auto_mode_uses_template_only_on_the_frame_after_semantic_failure(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.semantic_lane_mode = "auto"
        pipeline._auto_template_next = False

        self.assertFalse(pipeline._should_run_template())
        pipeline._auto_template_next = True
        self.assertTrue(pipeline._should_run_template())

    def test_semantic_bev_renders_warped_edges_and_gate_colours(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.selector = type("Selector", (), {"canvas_w": 100, "canvas_h": 100})()
        edge = np.zeros((100, 100), dtype=np.uint8)
        edge[10, 10] = 255
        result = {
            "edge_mask": edge,
            "accepted_bev_lines": [np.array([[20.0, 20.0], [20.0, 80.0]])],
            "rejected_bev_lines": [np.array([[70.0, 20.0], [70.0, 80.0]])],
        }

        image = pipeline._draw_semantic_bev(result, np.eye(3, dtype=np.float64))

        self.assertTrue(np.all(image[10, 10] > 0))
        self.assertGreater(int(image[50, 20, 1]), int(image[50, 20, 2]))
        self.assertGreater(int(image[50, 70, 2]), int(image[50, 70, 1]))

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
        cv2.rectangle(mask, (155, 120), (165, 360), 255, -1)
        cv2.rectangle(mask, (345, 120), (355, 360), 255, -1)
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
        cv2.line(mask, (155, 360), (210, 120), 255, 8)
        cv2.line(mask, (345, 360), (345, 120), 255, 8)

        result = SemanticLaneDetector(
            pixel_per_mm=1.0,
            bev_width=512,
            lane_width_mm=190.0,
            distance_tolerance_mm=15.0,
            parallel_tolerance_deg=3.0,
        ).analyze(mask, np.eye(3, dtype=np.float64))

        self.assertFalse(result["accepted"])
        self.assertEqual(result["fallback_reason"], "no_parallel_distance_valid_pair")


if __name__ == "__main__":
    unittest.main()
