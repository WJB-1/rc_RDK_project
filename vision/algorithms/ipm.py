# -*- coding: utf-8 -*-
"""
数学逆透视变换 (Analytical IPM)

基于相机内外参推导 M_IPM = M_scale @ inv(K @ E)，
将透视图像映射到 BEV 物理鸟瞰图（单位: mm）。

输入: 相机内参(f, c_x, c_y) + 外参(pitch, height) + 画布参数
输出: M_IPM 矩阵, warp() 方法
"""

import math
import numpy as np
import cv2


class MathematicalIPM:
    """基于相机内外参的解析逆透视变换"""

    def __init__(
        self,
        img_w: int = 1920,
        img_h: int = 1080,
        focal_length_mm: float = 2.8,
        pixel_size_mm: float = 0.003,
        camera_height_mm: float = 190.0,
        pitch_deg: float = 40.0,
        canvas_w: int = 600,
        canvas_h: int = 1200,
        pixel_per_mm: float = 0.5,
        blind_spot_mm: float = 0.0,
    ):
        """
        预计算 M_IPM 矩阵（初始化时只执行一次）。

        BEV 画布布局:
          画布底边 (y = new_canvas_h) 严格对应车头前保险杠 (物理 Y = 0)。
          盲区自动推导: Y_near = h / tan(θ + α), α = atan(img_h / 2f)
          向下扩建盲区像素 blind_spot_px，使最近可见地面映射到画布上方。

        矩阵推导:
          K     — 相机内参
          E     — 相机→地面外参映射
          H     = K @ E
          M_scale = [[ppm, 0, cw/2], [0, -ppm, new_ch], [0, 0, 1]]
          M_IPM   = M_scale @ inv(H)
        """
        f_y = focal_length_mm / pixel_size_mm

        # 自动计算盲区深度
        if blind_spot_mm <= 0.0:
            theta = math.radians(pitch_deg)
            alpha = math.atan(img_h / (2.0 * f_y))
            blind_spot_mm = camera_height_mm / math.tan(theta + alpha)
            blind_spot_mm = max(0.0, blind_spot_mm)

        self.pixel_per_mm = pixel_per_mm
        self.camera_height_mm = camera_height_mm
        self.pitch_deg = pitch_deg
        self.blind_spot_mm = blind_spot_mm
        self.blind_spot_px = int(blind_spot_mm * pixel_per_mm)
        self.original_canvas_h = canvas_h
        self.new_canvas_h = canvas_h + self.blind_spot_px
        self.canvas_size = (canvas_w, self.new_canvas_h)

        # K
        f_x = f_y
        c_x = img_w / 2.0
        c_y = img_h / 2.0
        K = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]], dtype=np.float64)

        # E
        theta = math.radians(pitch_deg)
        h = camera_height_mm
        E = np.array([
            [1, 0,                      0],
            [0, -math.sin(theta), h * math.cos(theta)],
            [0,  math.cos(theta), h * math.sin(theta)],
        ], dtype=np.float64)

        H = K @ E

        # M_scale: 物理尺度 → BEV 像素
        ppm = pixel_per_mm
        M_scale = np.array([
            [ppm, 0,    canvas_w / 2.0],
            [0,   -ppm, self.new_canvas_h],
            [0,   0,    1],
        ], dtype=np.float64)

        self.M_IPM = M_scale @ np.linalg.inv(H)

    def warp(self, clean_mask: np.ndarray) -> np.ndarray:
        """对清洗后的二值掩码做逆透视变换 → BEV 鸟瞰图"""
        return cv2.warpPerspective(
            clean_mask, self.M_IPM, self.canvas_size, flags=cv2.INTER_NEAREST
        )
