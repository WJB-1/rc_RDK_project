import unittest


class VisionOnlyStartTests(unittest.TestCase):
    def test_entrypoint_enables_vision_only_mode(self):
        from start_vision import create_runner

        runner = create_runner()

        self.assertTrue(runner.vision_only)


if __name__ == "__main__":
    unittest.main()
