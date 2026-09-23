"""Horizon BPU decoder for the single-class YOLOv8 road segmentation model."""

import cv2
import numpy as np

from .bisenet import _bgr2nv12


def _sigmoid(values):
    values = np.clip(values, -60.0, 60.0)
    return 1.0 / (1.0 + np.exp(-values))


def _nms(boxes, scores, iou_threshold):
    order = np.argsort(scores)[::-1]
    keep = []
    while len(order):
        current = order[0]
        keep.append(current)
        if len(order) == 1:
            break
        remaining = order[1:]
        first = boxes[current]
        others = boxes[remaining]
        left = np.maximum(first[0], others[:, 0])
        top = np.maximum(first[1], others[:, 1])
        right = np.minimum(first[2], others[:, 2])
        bottom = np.minimum(first[3], others[:, 3])
        intersection = np.maximum(0.0, right - left) * np.maximum(0.0, bottom - top)
        union = ((first[2] - first[0]) * (first[3] - first[1])
                 + (others[:, 2] - others[:, 0]) * (others[:, 3] - others[:, 1]) - intersection)
        order = remaining[intersection / np.maximum(union, 1e-6) <= iou_threshold]
    return np.asarray(keep, dtype=np.int32)


def decode_yolov8_seg_outputs(predictions, prototypes, image_size,
                               confidence_threshold=0.25, iou_threshold=0.7,
                               mask_threshold=0.5):
    """Decode YOLOv8-seg BPU outputs for square or rectangular inputs."""
    image_width, image_height = image_size
    predictions = np.asarray(predictions, dtype=np.float32).reshape(37, -1).T
    prototype_values = np.asarray(prototypes, dtype=np.float32)
    expected_proto_height = image_height // 4
    expected_proto_width = image_width // 4
    expected_proto_size = 32 * expected_proto_height * expected_proto_width
    if prototype_values.size != expected_proto_size:
        raise ValueError(
            "unexpected YOLOv8-seg prototype size: "
            f"got {prototype_values.size}, expected {expected_proto_size} "
            f"for input {image_width}x{image_height}"
        )
    prototypes = prototype_values.reshape(32, expected_proto_height, expected_proto_width)
    scores = predictions[:, 4]
    if scores.min() < 0.0 or scores.max() > 1.0:
        scores = _sigmoid(scores)
    selected = np.flatnonzero(scores >= confidence_threshold)
    if not len(selected):
        return np.zeros((image_height, image_width), dtype=np.uint8), []

    candidates = predictions[selected]
    centers = candidates[:, :2]
    sizes = np.maximum(candidates[:, 2:4], 0.0)
    boxes = np.concatenate((centers - sizes * 0.5, centers + sizes * 0.5), axis=1)
    boxes[:, (0, 2)] = np.clip(boxes[:, (0, 2)], 0, image_width)
    boxes[:, (1, 3)] = np.clip(boxes[:, (1, 3)], 0, image_height)
    kept = _nms(boxes, scores[selected], iou_threshold)

    proto_height, proto_width = prototypes.shape[1:]
    proto_flat = prototypes.reshape(32, -1)
    combined = np.zeros((image_height, image_width), dtype=np.uint8)
    detections = []
    for local_index in kept:
        coefficients = candidates[local_index, 5:37]
        probability = _sigmoid(coefficients @ proto_flat).reshape(proto_height, proto_width)
        x1, y1, x2, y2 = boxes[local_index]
        mask_x1 = max(0, int(np.floor(x1 * proto_width / image_width)))
        mask_y1 = max(0, int(np.floor(y1 * proto_height / image_height)))
        mask_x2 = min(proto_width, int(np.ceil(x2 * proto_width / image_width)))
        mask_y2 = min(proto_height, int(np.ceil(y2 * proto_height / image_height)))
        cropped = np.zeros_like(probability)
        cropped[mask_y1:mask_y2, mask_x1:mask_x2] = probability[mask_y1:mask_y2, mask_x1:mask_x2]
        full_mask = cv2.resize(cropped, (image_width, image_height), interpolation=cv2.INTER_LINEAR)
        combined[full_mask >= mask_threshold] = 255
        detections.append({"box_xyxy": boxes[local_index].tolist(), "score": float(scores[selected][local_index])})
    return combined, detections


class YoloRoadSegmentationEngine:
    """BPU inference wrapper returning one binary drivable-area mask per frame."""

    def __init__(self, model_path, input_size=(640, 640), confidence_threshold=0.25,
                 iou_threshold=0.7, mask_threshold=0.5):
        try:
            from hobot_dnn import pyeasy_dnn as dnn
        except ImportError as error:
            raise ImportError("YOLOv8-seg BPU inference requires hobot_dnn on RDK") from error
        models = dnn.load(str(model_path))
        if not models:
            raise RuntimeError(f"unable to load YOLOv8-seg model: {model_path}")
        self.model = models[0]
        if isinstance(input_size, (list, tuple)):
            self.input_width, self.input_height = map(int, input_size)
        else:
            self.input_width = self.input_height = int(input_size)
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.mask_threshold = float(mask_threshold)
        self.last_detections = []

    def inference(self, frame):
        height, width = frame.shape[:2]
        scale = min(self.input_width / width, self.input_height / height)
        resized_size = (round(width * scale), round(height * scale))
        resized = cv2.resize(frame, resized_size, interpolation=cv2.INTER_AREA)
        top = (self.input_height - resized_size[1]) // 2
        left = (self.input_width - resized_size[0]) // 2
        letterboxed = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
        letterboxed[top:top + resized_size[1], left:left + resized_size[0]] = resized
        outputs = self.model.forward([_bgr2nv12(letterboxed)])
        mask, self.last_detections = decode_yolov8_seg_outputs(
            outputs[0].buffer, outputs[1].buffer, (self.input_width, self.input_height),
            self.confidence_threshold, self.iou_threshold, self.mask_threshold,
        )
        content = mask[top:top + resized_size[1], left:left + resized_size[0]]
        return cv2.resize(content, (width, height), interpolation=cv2.INTER_NEAREST)
