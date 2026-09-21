# perception/algorithms/lane/ground_ipm_projector.py
"""Ground-IPM 投影器：图像 ↔ BEV 的矩阵构建与双向变换。

把原 GroundIPMLanePairSelector 内的：
  - _build_matrix
  - _matrix_for_profile
  - _project_line
  - _unproject_segment
抽离为独立类，selector 通过组合持有它。

不持有任何算法状态，不涉及可视化。
"""
import math

import cv2
import numpy as np


class GroundIPMProjector:
    """地面 IPM 投影器。

    负责：
      - 由相机内外参 + 画布参数构建 IPM 矩阵
      - 维护 lane / wall_outer 两套 profile 矩阵
      - 线段图像 ↔ BEV 双向变换
    """

    def __init__(self, input_size, calibration_size,
                 fx_px, fy_px, cx_px, cy_px,
                 camera_height_mm, pitch_deg,
                 canvas_w, canvas_h_base,
                 pixel_per_mm, blind_spot_mm,
                 wall_height_mm=50.0):
        self.input_width, self.input_height = map(int, input_size)
        self.calibration_width, self.calibration_height = map(int, calibration_size)
        self.canvas_w = int(canvas_w)
        self.canvas_h_base = int(canvas_h_base)
        self.pixel_per_mm = float(pixel_per_mm)
        self.blind_spot_mm = float(blind_spot_mm)
        self.blind_spot_px = int(round(self.blind_spot_mm * self.pixel_per_mm))
        self.canvas_h = self.canvas_h_base + self.blind_spot_px

        self.camera_height_mm = float(camera_height_mm)
        self.pitch_deg = float(pitch_deg)
        self.wall_height_mm = float(wall_height_mm)

        # lane 矩阵
        self.matrix = self._build_matrix(
            fx_px, fy_px, cx_px, cy_px, camera_height_mm, pitch_deg,
        )
        self._ground_matrix = self.matrix.copy()
        # wall_outer 矩阵：抬高虚拟相机高度
        self._profile_matrices = {
            "lane": self._ground_matrix,
            "wall_outer": self._build_matrix(
                fx_px, fy_px, cx_px, cy_px,
                float(camera_height_mm) - self.wall_height_mm,
                pitch_deg,
            ),
        }

    def _build_matrix(self, fx_px, fy_px, cx_px, cy_px, height_mm, pitch_deg):
        scale_x = self.input_width / float(self.calibration_width)
        scale_y = self.input_height / float(self.calibration_height)
        intrinsic = np.array([
            [float(fx_px) * scale_x, 0.0, float(cx_px) * scale_x],
            [0.0, float(fy_px) * scale_y, float(cy_px) * scale_y],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        pitch_rad = math.radians(float(pitch_deg))
        exterior = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -math.sin(pitch_rad), float(height_mm) * math.cos(pitch_rad)],
            [0.0, math.cos(pitch_rad), float(height_mm) * math.sin(pitch_rad)],
        ], dtype=np.float64)
        scale = np.array([
            [self.pixel_per_mm, 0.0, self.canvas_w / 2.0],
            [0.0, -self.pixel_per_mm, self.canvas_h_base + self.blind_spot_px],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        return scale @ np.linalg.inv(intrinsic @ exterior)

    def matrix_for_profile(self, profile_name):
        """取某 profile 的矩阵。兼容原实现：如果 self.matrix 被外部改写，优先用它。"""
        if not np.allclose(self.matrix, self._ground_matrix):
            return self.matrix
        return self._profile_matrices.get(profile_name, self.matrix)

    def project_line(self, line, matrix=None):
        """图像线段 → BEV 两点 [P1, P2]（np.ndarray shape (2, 2)）。"""
        points = np.array(
            [[[line["x1"], line["y1"]]], [[line["x2"], line["y2"]]]],
            dtype=np.float32,
        )
        return cv2.perspectiveTransform(
            points, self.matrix if matrix is None else matrix
        ).reshape(-1, 2)

    def unproject_segment(self, segment, matrix=None):
        """BEV 线段 → 图像线段 dict。"""
        points = np.asarray(segment, dtype=np.float32).reshape(1, 2, 2)
        active_matrix = self.matrix if matrix is None else matrix
        image_points = cv2.perspectiveTransform(points, np.linalg.inv(active_matrix))[0]
        return {
            "x1": float(image_points[0][0]), "y1": float(image_points[0][1]),
            "x2": float(image_points[1][0]), "y2": float(image_points[1][1]),
        }