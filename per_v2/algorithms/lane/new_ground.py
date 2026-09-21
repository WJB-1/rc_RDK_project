# perception/algorithms/lane/new_ground.py
import math
import numpy as np

from .config import CameraConfig


class NewGroundProjector:
    """地面系投影：使用独立的 ground_camera 参数。

    与 IPM 解耦，专门用于偏航角与横向偏移量的最终计算。
    内部把标定尺寸缩放到网络输入尺寸（512×384）。
    """

    def __init__(self, camera: CameraConfig, input_width: int, input_height: int):
        self.input_width = int(input_width)
        self.input_height = int(input_height)

        # 标定尺寸 → 输入尺寸
        sx = self.input_width / float(camera.img_w)
        sy = self.input_height / float(camera.img_h)

        self.fx = camera.fx_px * sx
        self.fy = camera.fy_px * sy
        self.cx = camera.cx_px * sx
        self.cy = camera.cy_px * sy

        self.camera_height_mm = float(camera.camera_height_mm)
        self.pitch_rad = math.radians(float(camera.pitch_deg))
        self.sin_pitch = math.sin(self.pitch_rad)
        self.cos_pitch = math.cos(self.pitch_rad)

    # ---------------------------------------------------------
    # 图像 ↔ 地面
    # ---------------------------------------------------------
    def image_to_ground(self, u: float, v: float):
        """图像点（input_size 坐标）→ 新地面系 (X_mm, Y_mm)。"""
        k = (v - self.cy) / self.fy
        denom = self.sin_pitch + k * self.cos_pitch
        if abs(denom) < 1e-9:
            return None
        y_mm = self.camera_height_mm * (self.cos_pitch - k * self.sin_pitch) / denom
        z_cam = self.cos_pitch * y_mm + self.camera_height_mm * self.sin_pitch
        if z_cam <= 1e-6:
            return None
        x_mm = (u - self.cx) / self.fx * z_cam
        return float(x_mm), float(y_mm)

    def ground_to_image(self, x_mm: float, y_mm: float):
        """新地面系 (X_mm, Y_mm) → 图像点（input_size 坐标）。"""
        denom = y_mm * self.cos_pitch + self.camera_height_mm * self.sin_pitch
        if abs(denom) < 1e-9:
            return None
        k = (self.camera_height_mm * self.cos_pitch - y_mm * self.sin_pitch) / denom
        z_cam = self.cos_pitch * y_mm + self.camera_height_mm * self.sin_pitch
        if z_cam <= 1e-6:
            return None
        v = self.cy + k * self.fy
        u = self.cx + x_mm * self.fx / z_cam
        return float(u), float(v)

    # ---------------------------------------------------------
    # 采样线
    # ---------------------------------------------------------
    def source_line_to_ground(self, line, samples: int = 40):
        """原图 Hough 线段 → 新地面系采样点列表 [(X, Y), ...]。"""
        if line is None:
            return []
        ts = np.linspace(0.0, 1.0, samples)
        pts = []
        for t in ts:
            u = line["x1"] + t * (line["x2"] - line["x1"])
            v = line["y1"] + t * (line["y2"] - line["y1"])
            g = self.image_to_ground(u, v)
            if g is not None:
                pts.append(g)
        return pts

    @staticmethod
    def fit_centerline(pts):
        """新地面系点 → x = a*y + b。返回 (a, b) 或 None。"""
        if len(pts) < 5:
            return None
        ys = np.array([p[1] for p in pts], dtype=np.float64)
        xs = np.array([p[0] for p in pts], dtype=np.float64)
        keep = (ys > 50.0) & (ys < 8000.0)
        if keep.sum() < 5:
            return None
        a, b = np.polyfit(ys[keep], xs[keep], 1)
        return float(a), float(b)