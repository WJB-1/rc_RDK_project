# -*- coding: utf-8 -*-
"""
路口检测 — 纯语义分割方案 (替代 IPM 占空比法)

原理:
  1. 路口 = 语义分割 mask 底部 ROI 中白色像素横向贯穿
  2. 距离 = 小孔成像相似三角形: distance = K / (y_px - y_horizon)

标定:
  在已知距离 D_cal mm 处放一条横线，记录其在图像中的像素行 y_cal，
  则 K = D_cal * (y_cal - y_horizon)。之后 K 为常数。

使用:
  from .crossroad_seg import detect_crossroad_from_seg, calibrate_pinhole
  # 一次标定
  calibrate_pinhole(y_cal=320, D_cal=1000.0, y_horizon=240, f_mm=3.6, sensor_h=2.74)
  # 运行时
  is_crossroad, distance_mm, duty = detect_crossroad_from_seg(seg_mask, roi_y1, roi_y2)
"""

import numpy as np

# ---- 可标定参数 ----
_PINHOLE_K = None          # K = D_cal * (y_cal - y_horizon)，标定后为常数
_Y_HORIZON = None          # 地平线像素行
_FOCAL_PX = None           # 焦距 (像素)，替代方案

# ---- 连续帧确认 ----
_CROSSROAD_COUNTER = 0
_CROSSROAD_CONFIRM_FRAMES = 3
_CROSSROAD_DECAY_FRAMES = 2


def calibrate_pinhole(y_cal: float, D_cal: float, y_horizon: float,
                      f_mm: float = None, sensor_h_mm: float = None,
                      image_h: int = None):
    """
    标定小孔成像参数。两种方式任选其一:

    方式A (推荐): 提供 y_cal, D_cal, y_horizon
      K = D_cal * (y_cal - y_horizon)
      距离 = K / (y_px - y_horizon)

    方式B: 提供 f_mm, sensor_h_mm, image_h, y_horizon
      focal_px = f_mm * image_h / sensor_h_mm
      距离 = focal_px * camera_height_mm / (y_px - y_horizon)  * camera_height 另行传入

    Args:
        y_cal: 标定横线在图像中的像素行
        D_cal: 标定时小车到横线的真实距离 (mm)
        y_horizon: 地平线在图像中的像素行
        f_mm: 相机物理焦距 (mm) — 方式B
        sensor_h_mm: 传感器物理高度 (mm) — 方式B
        image_h: 图像像素高度 — 方式B
    """
    global _PINHOLE_K, _Y_HORIZON, _FOCAL_PX

    _Y_HORIZON = y_horizon

    if y_cal is not None and D_cal is not None:
        _PINHOLE_K = D_cal * (y_cal - y_horizon)

    if f_mm is not None and sensor_h_mm is not None and image_h is not None:
        _FOCAL_PX = f_mm * image_h / sensor_h_mm


def estimate_distance(y_px: float, camera_height_mm: float = 150.0) -> float:
    """
    小孔成像距离估算。

    Args:
        y_px: 检测到的横线在图像中的像素行（越靠近画面底部值越大）
        camera_height_mm: 相机光心离地高度 (mm)

    Returns:
        到横线的估算距离 (mm)。未标定时返回 -1。
    """
    if _PINHOLE_K is not None and _Y_HORIZON is not None:
        dy = y_px - _Y_HORIZON
        if dy > 1.0:
            return _PINHOLE_K / dy

    if _FOCAL_PX is not None and _Y_HORIZON is not None:
        dy = y_px - _Y_HORIZON
        if dy > 1.0:
            return _FOCAL_PX * camera_height_mm / dy

    return -1.0


def detect_crossroad_from_seg(seg_mask: np.ndarray,
                               roi_y1: int,
                               roi_y2: int,
                               camera_height_mm: float = 150.0,
                               duty_threshold: float = 0.75):
    """
    从语义分割 mask 直接检测路口。

    不依赖 IPM。在分割 mask 底部 ROI 区域计算:
      - 白色像素水平跨度 / ROI 宽度 = 占空比
      - 占空比 > duty_threshold → 检测到路口横线
      - 横线像素行 → 小孔成像估算距离

    Args:
        seg_mask: 语义分割二值 mask (H, W)，uint8，255=车道线
        roi_y1: ROI 区域起始行 (靠近图像顶部)
        roi_y2: ROI 区域结束行 (靠近图像底部)
        camera_height_mm: 相机离地高度 (mm)
        duty_threshold: 占空比阈值 (0.0~1.0)

    Returns:
        (is_crossroad: bool, distance_mm: float, duty_cycle: float)
    """
    h, w = seg_mask.shape[:2]
    y1 = max(0, roi_y1)
    y2 = min(h, roi_y2)

    if y2 <= y1:
        return False, -1.0, 0.0

    window = seg_mask[y1:y2, :]
    white_indices = np.where(window == 255)[1]

    if white_indices.size == 0:
        return False, -1.0, 0.0

    left_x = int(white_indices.min())
    right_x = int(white_indices.max())
    horizontal_span = right_x - left_x
    duty_cycle = horizontal_span / w

    is_crossroad = duty_cycle > duty_threshold

    distance_mm = -1.0
    if is_crossroad:
        # 横线的像素行：取白色像素最靠底部的那一行
        white_rows = np.where(window == 255)[0]
        crossroad_y_px = float(y1 + white_rows.max())
        distance_mm = estimate_distance(crossroad_y_px, camera_height_mm)
        # 未标定时的兜底
        if distance_mm <= 0:
            distance_mm = 150.0

    return is_crossroad, distance_mm, duty_cycle


def confirm_crossroad_seg(is_crossroad_frame: bool):
    """
    连续帧确认: 需要连续 _CROSSROAD_CONFIRM_FRAMES 帧都检测到路口才确认。
    非路口时计数衰减但不归零（防止单帧漏检）。

    Returns:
        (confirmed: bool, counter: int)
    """
    global _CROSSROAD_COUNTER

    if is_crossroad_frame:
        _CROSSROAD_COUNTER += 1
    else:
        _CROSSROAD_COUNTER = max(0, _CROSSROAD_COUNTER - _CROSSROAD_DECAY_FRAMES)

    return _CROSSROAD_COUNTER >= _CROSSROAD_CONFIRM_FRAMES, _CROSSROAD_COUNTER


def reset_crossroad_counter_seg():
    """转弯完成时调用，重置确认计数器"""
    global _CROSSROAD_COUNTER
    _CROSSROAD_COUNTER = 0
