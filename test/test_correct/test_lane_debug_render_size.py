import unittest


class LaneDebugRenderSizeTests(unittest.TestCase):
    def test_new_ground_view_matches_processing_image_size(self):
        from perception.diagnostics.lane_selector_viz import LaneSelectorRenderer

        selector = type("Selector", (), {
            "input_width": 512,
            "input_height": 384,
            "canvas_w": 400,
            "canvas_h": 400,
        })()

        image = LaneSelectorRenderer(selector).render_new_ground_bev()

        self.assertEqual(image.shape, (384, 512, 3))

    def test_parallel_view_matches_processing_image_size(self):
        from perception.algorithms.lane.pipeline import LanePipeline

        pipeline = LanePipeline.__new__(LanePipeline)
        pipeline.selector = type("Selector", (), {
            "input_width": 512,
            "input_height": 384,
            "canvas_w": 400,
            "canvas_h": 400,
            "_last_parallel_groups": [],
        })()

        image = pipeline._draw_parallel_groups()

        self.assertEqual(image.shape, (384, 512, 3))


if __name__ == "__main__":
    unittest.main()
