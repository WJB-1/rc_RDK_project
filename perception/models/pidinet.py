from pathlib import Path
import time
from perception.algorithms.core.timing import block
import cv2
import numpy as np

from perception.algorithms.lane.line_detection import (
    detect_lines,
    draw_lines,
    merge_lines,
    postprocess_edge_probability,
    thin_binary,
)


def bgr_to_nv12(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    area = height * width
    yuv420 = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape(area * 3 // 2)
    y_plane = yuv420[:area]
    uv_planar = yuv420[area:].reshape(2, area // 4)
    uv_interleaved = uv_planar.T.reshape(area // 2)
    result = np.empty_like(yuv420)
    result[:area] = y_plane
    result[area:] = uv_interleaved
    return result


def enhance_image(image: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    lightness, channel_a, channel_b = cv2.split(lab)
    lightness = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(lightness)
    enhanced = cv2.cvtColor(cv2.merge((lightness, channel_a, channel_b)), cv2.COLOR_LAB2BGR)
    blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.2)
    return cv2.addWeighted(enhanced, 1.35, blurred, -0.35, 0)


def restore_mask_to_frame(mask: np.ndarray, frame_shape) -> np.ndarray:
    """Map PiDiNet coordinates back to calibrated camera pixels."""
    height, width = frame_shape[:2]
    return cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)


def scale_lines_to_frame(lines, source_size, frame_shape):
    source_width, source_height = source_size
    frame_height, frame_width = frame_shape[:2]
    scale_x = frame_width / float(source_width)
    scale_y = frame_height / float(source_height)
    scaled = []
    for line in lines:
        item = dict(line)
        item["x1"] *= scale_x
        item["x2"] *= scale_x
        item["y1"] *= scale_y
        item["y2"] *= scale_y
        scaled.append(item)
    return scaled


class PiDiNetEngine:
    def __init__(self, model_path: str, input_size=(512, 384), edge_threshold=0.25,
                 angle_tol_deg=3.0, normal_dist_tol=8.0):
        self.model_path = str(Path(model_path))
        self.input_width, self.input_height = input_size
        self.edge_threshold = float(edge_threshold)
        self.angle_tol_deg = float(angle_tol_deg)
        self.normal_dist_tol = float(normal_dist_tol)
        try:
            from hobot_dnn import pyeasy_dnn as dnn
        except ImportError as error:
            raise ImportError("PiDiNet BPU 推理需要在 RDK X5 环境安装 hobot_dnn") from error
        models = dnn.load(self.model_path)
        if not models:
            raise RuntimeError(f"无法加载 PiDiNet 模型: {self.model_path}")
        self.model = models[0]
        self.last_probability = None
        self.last_enhanced = None
        self.last_binary = None
        self.last_closed = None
        self.last_skeleton = None
        self.last_label_map = None
        self.last_n_labels = 0
        self.last_raw_lines = []
        self.last_lines = []
        self.last_inference_ms = 0.0
        self.last_postprocess_ms = 0.0

    def _read_probability(self, output):
        probability = np.asarray(output, dtype=np.float32).squeeze()
        expected_shape = (self.input_height, self.input_width)
        if probability.size != self.input_height * self.input_width:
            raise RuntimeError(f"PiDiNet 输出尺寸异常: {probability.shape}")
        probability = probability.reshape(expected_shape)
        if probability.min() < 0.0 or probability.max() > 1.0:
            probability = 1.0 / (1.0 + np.exp(-probability))
        return probability

    def inference(self, frame: np.ndarray) -> np.ndarray:
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            raise ValueError("输入帧无效")
        inference_started = time.perf_counter()

        with block("edge.resize"):
            if frame.shape[:2] == (self.input_height, self.input_width):
                resized = frame
            else:
                resized = cv2.resize(
                    frame,
                    (self.input_width, self.input_height),
                    interpolation=cv2.INTER_AREA,
                )
        with block("edge.enhance"):
            enhanced = enhance_image(resized)
        with block("edge.bgr2nv12"):
            nv12 = bgr_to_nv12(enhanced)
        with block("edge.bpu_forward"):
            output = self.model.forward([nv12])[0].buffer
        with block("edge.probability"):
            probability = self._read_probability(output)
        self.last_inference_ms = (time.perf_counter() - inference_started) * 1000.0

        postprocess_started = time.perf_counter()
        with block("edge.threshold"):
            thresholded = (probability >= self.edge_threshold).astype(np.uint8) * 255
        with block("edge.close"):
            closed = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        with block("edge.thin"):
            binary = thin_binary(closed)
        with block("edge.hough"):
            raw_lines = detect_lines(binary)
        with block("edge.merge"):
            merged_lines = merge_lines(
                raw_lines,
                angle_tol_deg=self.angle_tol_deg,
                normal_dist_tol=self.normal_dist_tol,
            )
        with block("edge.connected_components"):
            n_labels, label_map = cv2.connectedComponents(binary)
        self.last_postprocess_ms = (time.perf_counter() - postprocess_started) * 1000.0

        self.last_probability = probability
        self.last_enhanced = enhanced
        self.last_binary = binary
        self.last_closed = closed
        self.last_skeleton = binary
        self.last_label_map = label_map
        self.last_n_labels = int(n_labels)
        self.last_raw_lines = raw_lines
        self.last_lines = merged_lines
        return binary
    
    def render_lines(self, frame: np.ndarray) -> np.ndarray:
        if frame is None:
            return None
        resized = cv2.resize(frame, (self.input_width, self.input_height), interpolation=cv2.INTER_AREA)
        return draw_lines(resized, self.last_lines)
