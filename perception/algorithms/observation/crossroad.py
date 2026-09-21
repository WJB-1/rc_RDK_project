# -*- coding: utf-8 -*-
"""
路口检测 (Crossroad Detection)

基于 IPM 鸟瞰图的横坐标占空比法：
  直道: 水平跨度 ≈ 车道宽 / 画布宽 → 占空比 ≈ 16%
  路口横向车道: 水平跨度占满画布 → 占空比 > 80%
  毛边/围栏: 水平跨度 < 67%，不到 80% 阈值

配合连续帧确认计数器 (_CROSSROAD_COUNTER) 防止单帧噪声误触发。
"""

import numpy as np


# 全局连续帧确认状态
_CROSSROAD_COUNTER = 0
_CROSSROAD_CONFIRM_FRAMES = 3  # 连续 3 帧触发才确认
_CROSSROAD_DECAY_FRAMES = 2    # 非路口时衰减步长
_LAST_CROSSROAD_DUTY = 0.0


def detect_crossroad(bev_mask, y_top, y_bottom, canvas_w):
    """
    基于车道水平跨度占空比的路口检测。

    原理:
      horizontal_span = right_x - left_x  (白色像素水平跨度)
      duty_cycle = horizontal_span / canvas_w
      阈值 > 0.8 → 横向车道（路口）

    Returns: (is_crossroad: bool, duty_cycle: float)
    """
    if y_bottom <= y_top:
        return False, 0.0

    window = bev_mask[max(0, y_top):min(bev_mask.shape[0], y_bottom), :]
    white_indices = np.where(window == 255)[1]
    if white_indices.size == 0:
        return False, 0.0

    left_x = int(white_indices.min())
    right_x = int(white_indices.max())
    horizontal_span = right_x - left_x
    duty_cycle = horizontal_span / canvas_w

    return duty_cycle > 0.8, duty_cycle


def confirm_crossroad(is_crossroad_frame: bool):
    """
    连续帧确认: 需要连续 _CROSSROAD_CONFIRM_FRAMES 帧都检测到路口才确认。
    非路口时计数衰减但不归零（防止单帧漏检）。

    Returns: (confirmed: bool, counter: int)
    """
    global _CROSSROAD_COUNTER, _LAST_CROSSROAD_DUTY

    if is_crossroad_frame:
        _CROSSROAD_COUNTER += 1
    else:
        _CROSSROAD_COUNTER = max(0, _CROSSROAD_COUNTER - _CROSSROAD_DECAY_FRAMES)

    return _CROSSROAD_COUNTER >= _CROSSROAD_CONFIRM_FRAMES, _CROSSROAD_COUNTER


def set_last_duty(duty_cycle: float):
    global _LAST_CROSSROAD_DUTY
    _LAST_CROSSROAD_DUTY = duty_cycle


def get_last_duty() -> float:
    return _LAST_CROSSROAD_DUTY


def reset_crossroad_counter():
    """转弯完成时调用"""
    global _CROSSROAD_COUNTER
    _CROSSROAD_COUNTER = 0
