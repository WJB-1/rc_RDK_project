import unittest

import numpy as np


class PerceptionCompatibilityTests(unittest.TestCase):
    def test_pipeline_resizes_before_undistort_and_skips_disabled_debug_work(self):
        import perception.algorithms.lane.pipeline as lane_pipeline
        from perception.algorithms.lane.pipeline import LanePipeline

        class Config:
            lane_width_mm = 200.0
            bev_cfg = {}
            matching_cfg = {}
            template_x_at_ref_mm = {}

        class Undistorter:
            received_shape = None

            def apply(self, frame):
                self.received_shape = frame.shape
                return frame

        class EdgeEngine:
            input_width = 4
            input_height = 3
            last_lines = []
            last_raw_lines = []
            last_inference_ms = 0.0
            last_postprocess_ms = 0.0

            def inference(self, frame):
                assert frame.shape == (3, 4, 3)
                return np.zeros((3, 4), dtype=np.uint8)

        class Selector:
            detected_source_lines = []

            def analyze(self, lines, semantic_mask=None):
                return {
                    "pid_error_mm": 0.0,
                    "crossroad_detected": False,
                    "vis_points": {},
                }

        class Ipm:
            pitch_deg = 40.0

        class Renderer:
            def draw_bev_view(self):
                return np.zeros((3, 4, 3), dtype=np.uint8)

            def draw_original_view(self, frame):
                return frame.copy()

        original_renderer = lane_pipeline._make_renderer
        lane_pipeline._make_renderer = lambda selector: Renderer()
        try:
            undistorter = Undistorter()
            pipeline = LanePipeline(
                Config(), undistorter, EdgeEngine(), None, Selector(), Ipm(), False,
                {"debug": {"timing_enabled": False}},
            )
            pipeline.set_debug_render_enabled(False)
            pipeline.set_debug_capture_enabled(False)

            result = pipeline.process(np.zeros((6, 8, 3), dtype=np.uint8))
        finally:
            lane_pipeline._make_renderer = original_renderer

        self.assertEqual(undistorter.received_shape, (3, 4, 3))
        self.assertEqual(result.debug_frame.shape, (3, 4, 3))

    def test_tracker_can_disable_debug_capture_through_public_api(self):
        from perception.algorithms.lane.tracker import LaneTracker

        class Pipeline:
            debug_capture_enabled = True

            def set_debug_capture_enabled(self, enabled):
                self.debug_capture_enabled = bool(enabled)
                return self.debug_capture_enabled

        tracker = LaneTracker.__new__(LaneTracker)
        tracker.pipeline = Pipeline()

        self.assertFalse(tracker.set_debug_capture_enabled(False))
        self.assertFalse(tracker.pipeline.debug_capture_enabled)

    def test_tracker_can_disable_debug_rendering_through_public_api(self):
        from perception.algorithms.lane.tracker import LaneTracker

        class Pipeline:
            debug_render_enabled = True

            def set_debug_render_enabled(self, enabled):
                self.debug_render_enabled = bool(enabled)
                return self.debug_render_enabled

        tracker = LaneTracker.__new__(LaneTracker)
        tracker.pipeline = Pipeline()

        self.assertFalse(tracker.set_debug_render_enabled(False))
        self.assertFalse(tracker.pipeline.debug_render_enabled)

    def test_lane_geometry_exports_remain_importable(self):
        from perception.algorithms.lane.line_geometry import find_valid_lane_pairs

        pairs = find_valid_lane_pairs([], pixel_per_mm=0.5, bev_width=400)

        self.assertEqual(pairs, [])

    def test_lane_undistort_module_exposes_parameter_builder(self):
        from perception.algorithms.lane.undistort import build_undistort_parameters

        camera_matrix, distortion = build_undistort_parameters(
            {"enabled": False}, (480, 640, 3)
        )

        self.assertIsNone(camera_matrix)
        self.assertIsNone(distortion)

    def test_preview_diagnostic_dependency_remains_importable(self):
        from perception.diagnostics.lane_debug_viz import VIEW_BUILDERS

        self.assertIsInstance(VIEW_BUILDERS, dict)
