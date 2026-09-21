import unittest
import sys
from pathlib import Path

import cv2
import numpy as np

TEST_CORRECT_ROOT = Path(__file__).resolve().parent
if str(TEST_CORRECT_ROOT) not in sys.path:
    sys.path.insert(0, str(TEST_CORRECT_ROOT))


class VisionPreviewTests(unittest.TestCase):
    def test_undistort_parameters_scale_intrinsics_to_input_frame(self):
        from perception.algorithms.lane.undistort import build_undistort_parameters

        camera_matrix, distortion = build_undistort_parameters(
            {
                "enabled": True,
                "image_width": 1920,
                "image_height": 1080,
                "fx_px": 1148.0,
                "fy_px": 1150.0,
                "cx_px": 948.0,
                "cy_px": 568.0,
                "distortion": [0.1, -0.2, 0.001, 0.002, -0.03],
            },
            (540, 960, 3),
        )

        self.assertEqual(camera_matrix.shape, (3, 3))
        self.assertAlmostEqual(float(camera_matrix[0, 0]), 574.0)
        self.assertAlmostEqual(float(camera_matrix[1, 1]), 575.0)
        self.assertAlmostEqual(float(camera_matrix[0, 2]), 474.0)
        self.assertAlmostEqual(float(camera_matrix[1, 2]), 284.0)
        self.assertEqual(distortion.shape, (5,))

    def test_web_semantic_gate_command_updates_tracker_and_status(self):
        from runner import DebugRunner

        class FakeTracker:
            semantic_gate_enabled = True

            def set_semantic_gate(self, enabled):
                self.semantic_gate_enabled = bool(enabled)
                return self.semantic_gate_enabled

        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: None)
        runner._vision_tracker = FakeTracker()

        response = runner._create_app().test_client().post(
            "/api/command", json={"command": "set_semantic_gate", "enabled": False}
        )

        self.assertEqual(response.status_code, 200)
        self.assertFalse(runner._vision_tracker.semantic_gate_enabled)
        self.assertFalse(runner.snapshot()["semantic_gate_enabled"])

    def test_debug_panel_accepts_float_crossroad_coordinate(self):
        from perception.diagnostics.ipm_drawer import draw_debug_panel

        panel = draw_debug_panel(
            clean_mask=np.zeros((120, 160), dtype=np.uint8),
            bev_mask=np.zeros((80, 80), dtype=np.uint8),
            lane_state={
                "crossroad_detected": True,
                "vis_points": {"crossroad_y": 35.5},
            },
        )

        self.assertEqual(panel.shape, (800, 800, 3))

    def test_render_preview_resizes_and_returns_jpeg(self):
        from vision_preview import render_preview

        raw = np.zeros((1080, 1920, 3), dtype=np.uint8)
        clean = np.zeros((540, 960), dtype=np.uint8)
        bev = np.zeros((400, 400), dtype=np.uint8)

        encoded = render_preview(
            "binary",
            raw,
            clean,
            bev,
            {"vis_points": {}},
            camera_pitch_deg=40.0,
            physical_track_width_mm=450.0,
            max_size=(640, 480),
        )

        image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertIsNotNone(image)
        self.assertLessEqual(image.shape[1], 640)
        self.assertLessEqual(image.shape[0], 480)

    def test_render_preview_supports_all_view_names(self):
        from vision_preview import VIEW_NAMES, render_preview

        raw = np.zeros((120, 160, 3), dtype=np.uint8)
        clean = np.zeros((120, 160), dtype=np.uint8)
        bev = np.zeros((80, 80), dtype=np.uint8)
        state = {"vis_points": {}, "pid_error_mm": 12.0}

        original_view = raw.copy()
        original_view[20:80, 20:80] = (0, 0, 255)
        self.assertEqual(
            set(VIEW_NAMES),
            {
                "overlay", "binary", "hough", "lane_bev", "ground_bev",
                "semantic_overlay", "semantic_bev", "semantic_ground",
            },
        )
        for view in VIEW_NAMES:
            encoded = render_preview(view, raw, clean, bev, state, 40.0, 450.0, original_view=original_view)
            self.assertTrue(encoded.startswith(b"\xff\xd8"), view)

    def test_overlay_preview_uses_annotated_original_view(self):
        from vision_preview import render_preview

        raw = np.zeros((120, 160, 3), dtype=np.uint8)
        original_view = raw.copy()
        original_view[40:80, 40:120] = (0, 0, 255)
        encoded = render_preview(
            "overlay", raw, np.zeros((120, 160), dtype=np.uint8),
            np.zeros((80, 80, 3), dtype=np.uint8), {"vis_points": {}}, 40.0, 200.0,
            original_view=original_view,
        )

        image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertGreater(int(image[60, 80, 2]), 100)

    def test_hough_fallback_scales_merged_lines_to_raw_frame(self):
        from vision_preview import render_preview

        raw = np.zeros((100, 200, 3), dtype=np.uint8)
        encoded = render_preview(
            "hough", raw, None, None, {}, diagnostics={
                "merged_lines": [{"x1": 25, "y1": 10, "x2": 75, "y2": 40}],
                "source_size": (100, 50),
            },
        )
        image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertGreater(int(image[50, 100, 2]), 100)

    def test_offline_page_only_lists_lane_debug_views_and_defaults_to_overlay(self):
        from offline_runner import OFFLINE_PAGE, OfflineVisionRunner

        expected_views = {"overlay", "binary", "hough", "lane_bev", "ground_bev"}
        self.assertEqual(
            set(__import__("re").findall(r'<option value="([^"]+)">', OFFLINE_PAGE)),
            expected_views,
        )

        runner = OfflineVisionRunner()
        runner._jpeg = lambda view: view.encode()
        response = runner.create_app().test_client().get("/api/vision")
        self.assertEqual(response.data, b"overlay")

    def test_web_endpoint_returns_selected_preview(self):
        from runner import DebugRunner

        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: None)
        raw = np.zeros((120, 160, 3), dtype=np.uint8)
        clean = np.zeros((120, 160), dtype=np.uint8)
        bev = np.zeros((80, 80), dtype=np.uint8)
        runner.update_vision_preview(raw, clean, bev, {"vis_points": {}}, 0.0, False, 2.0)

        response = runner._create_app().test_client().get("/api/vision?view=overlay")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.mimetype, "image/jpeg")
        self.assertTrue(response.data.startswith(b"\xff\xd8"))

    def test_status_exposes_vision_timing_and_lane_metrics(self):
        from runner import DebugRunner

        runner = DebugRunner("loopback", 115200, 5002, serial_factory=lambda *args, **kwargs: None)
        raw = np.zeros((120, 160, 3), dtype=np.uint8)
        runner.update_vision_preview(
            raw, np.zeros((120, 160), dtype=np.uint8), np.zeros((80, 80), dtype=np.uint8),
            {"lane_angle_rad": 0.1, "pid_error_mm": 12.5, "quality_score": 0.8},
            12.5, False, 23.0,
            diagnostics={"debug_capture": {"metrics": {"raw_hough_count": 8, "template_confidence": 0.8}}},
            timing={"total_ms": 23.0, "inference_ms": 11.0, "postprocess_ms": 6.0},
        )

        payload = runner._create_app().test_client().get("/api/status").get_json()

        self.assertEqual(payload["vision_diagnostics"]["timing_ms"]["inference_ms"], 11.0)
        self.assertEqual(payload["vision_diagnostics"]["metrics"]["raw_hough_count"], 8)
        self.assertEqual(payload["vision_diagnostics"]["lane"]["offset_mm"], 12.5)


if __name__ == "__main__":
    unittest.main()
