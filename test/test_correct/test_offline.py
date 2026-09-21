import unittest


class OfflinePreviewTests(unittest.TestCase):
    def test_jpeg_passes_preview_fields_in_render_order(self):
        import offline_runner
        from offline_runner import OfflineVisionRunner

        runner = OfflineVisionRunner()
        runner._preview = ("raw", "clean", "bev", {}, 12.0, False, 33.0, 40.0, 450.0)
        captured = []
        original = offline_runner.render_preview
        offline_runner.render_preview = lambda *args: captured.append(args) or b"jpeg"
        try:
            self.assertEqual(runner._jpeg("binary"), b"jpeg")
        finally:
            offline_runner.render_preview = original

        self.assertEqual(captured[0], ("binary", "raw", "clean", "bev", {}, 40.0, 450.0, 12.0, 33.0))

    def test_offline_app_has_no_handshake_or_serial_controls(self):
        from offline_runner import OfflineVisionRunner

        runner = OfflineVisionRunner(web_port=5010)
        html = runner.create_app().test_client().get("/").data.decode("utf-8")

        self.assertIn("单机视觉回正", html)
        self.assertNotIn("连接并握手", html)
        self.assertIn("/api/vision", html)


if __name__ == "__main__":
    unittest.main()
