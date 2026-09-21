import unittest

import cv2
import numpy as np


class SemanticLaneTests(unittest.TestCase):
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
