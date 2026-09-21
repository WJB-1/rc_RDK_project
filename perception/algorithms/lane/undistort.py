# perception/algorithms/lane/undistort.py
import cv2
import numpy as np


def build_undistort_parameters(calibration: dict, frame_shape):
    if not calibration or not calibration.get("enabled", False):
        return None, None

    height, width = frame_shape[:2]
    source_width = float(calibration.get("image_width", width))
    source_height = float(calibration.get("image_height", height))
    scale_x = width / source_width
    scale_y = height / source_height

    camera_matrix = np.array([
        [float(calibration["fx_px"]) * scale_x, 0.0, float(calibration["cx_px"]) * scale_x],
        [0.0, float(calibration["fy_px"]) * scale_y, float(calibration["cy_px"]) * scale_y],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    distortion = np.asarray(
        calibration.get("distortion", [0, 0, 0, 0, 0]),
        dtype=np.float64,
    ).reshape(-1)
    if distortion.size < 5:
        distortion = np.pad(distortion, (0, 5 - distortion.size))
    return camera_matrix, distortion[:5]


class Undistorter:
    def __init__(self, calibration: dict):
        self.calibration = calibration or {}
        self._cache = None

    def apply(self, frame: np.ndarray) -> np.ndarray:
        if not self.calibration.get("enabled", False):
            return frame

        h, w = frame.shape[:2]
        if self._cache is not None and self._cache[2] == (w, h):
            map1, map2, _ = self._cache
        else:
            camera_matrix, distortion = build_undistort_parameters(self.calibration, frame.shape)
            if camera_matrix is None:
                self._cache = (None, None, (w, h))
                return frame
            map1, map2 = cv2.initUndistortRectifyMap(
                camera_matrix, distortion, None, camera_matrix,
                (w, h), cv2.CV_16SC2,
            )
            self._cache = (map1, map2, (w, h))

        if map1 is None:
            return frame
        return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)