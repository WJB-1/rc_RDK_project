import unittest

import numpy as np


class YoloRoadSegTests(unittest.TestCase):
    def test_prototype_shape_reveals_square_compiled_model(self):
        from perception.models.yolo_road_seg import _prototype_spatial_shape

        flattened = np.zeros(32 * 160 * 160, dtype=np.float32)

        self.assertEqual(_prototype_spatial_shape(flattened, (640, 480)), (160, 160))

    def test_decoder_supports_640_by_480_model_outputs(self):
        from perception.models.yolo_road_seg import decode_yolov8_seg_outputs

        predictions = np.zeros((1, 37, 6300, 1), dtype=np.float32)
        predictions[0, 4, 0, 0] = 0.9
        predictions[0, 0:4, 0, 0] = [320.0, 240.0, 200.0, 120.0]
        predictions[0, 5, 0, 0] = 10.0
        prototypes = np.zeros((1, 32, 120, 160), dtype=np.float32)
        prototypes[0, 0, 45:75, 55:105] = 1.0

        mask, detections = decode_yolov8_seg_outputs(
            predictions, prototypes, (640, 480), confidence_threshold=0.25,
        )

        self.assertEqual(mask.shape, (480, 640))
        self.assertEqual(len(detections), 1)
        self.assertGreater(int(np.count_nonzero(mask)), 0)

    def test_decodes_single_road_instance_mask(self):
        from perception.models.yolo_road_seg import decode_yolov8_seg_outputs

        predictions = np.zeros((1, 37, 8400, 1), dtype=np.float32)
        predictions[0, :4, 0, 0] = (320.0, 320.0, 320.0, 320.0)
        predictions[0, 4, 0, 0] = 0.9
        predictions[0, 5, 0, 0] = 8.0
        prototypes = np.zeros((1, 32, 160, 160), dtype=np.float32)
        prototypes[0, 0] = 1.0

        mask, detections = decode_yolov8_seg_outputs(
            predictions, prototypes, (640, 640), confidence_threshold=0.25, iou_threshold=0.7,
        )

        self.assertEqual(len(detections), 1)
        self.assertEqual(mask.shape, (640, 640))
        self.assertGreater(np.count_nonzero(mask), 100000)

    def test_rejects_overlapping_lower_confidence_detection(self):
        from perception.models.yolo_road_seg import decode_yolov8_seg_outputs

        predictions = np.zeros((1, 37, 8400, 1), dtype=np.float32)
        for index, score in enumerate((0.9, 0.5)):
            predictions[0, :4, index, 0] = (320.0, 320.0, 300.0, 300.0)
            predictions[0, 4, index, 0] = score
            predictions[0, 5, index, 0] = 8.0
        prototypes = np.zeros((1, 32, 160, 160), dtype=np.float32)
        prototypes[0, 0] = 1.0

        _, detections = decode_yolov8_seg_outputs(predictions, prototypes, (640, 640))

        self.assertEqual(len(detections), 1)
