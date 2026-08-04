# -*- coding: utf-8 -*-
"""
车道状态分析器 (Lane State Analyzer)

三段式滑动窗口状态机核心:
  阶段1: 从 BEV 底部向上扫描, 提取 3~5 个近端锚点
  阶段2: 外推物理盲区, 得到车头横向偏移
  阶段3: 向上侦测, 路口由 crossroad_detect 模块判定

输出严格解耦的控制接口字典。
"""

import math
import numpy as np

from .quality_gate import validate_frame_quality
from .crossroad_detect import (
    detect_crossroad, confirm_crossroad, set_last_duty
)

# 丢帧保护
_CONSECUTIVE_DROP_COUNT = 0
_MAX_CONSECUTIVE_DROPS = 15
_LAST_VALID_STATE = None

# 对外暴露的路口计数重置（转弯完成时调用）
from .crossroad_detect import reset_crossroad_counter


def reset_drop_counter():
    global _CONSECUTIVE_DROP_COUNT
    _CONSECUTIVE_DROP_COUNT = 0


def _empty_lane_state(quality_score=0.0, frame_dropped=False, drop_reason=""):
    return {
        "pid_error_mm": 0.0,
        "crossroad_detected": False,
        "distance_to_crossroad_mm": -1.0,
        "lane_angle_rad": 0.0,
        "quality_score": quality_score,
        "frame_dropped": frame_dropped,
        "drop_reason": drop_reason,
        "duty_cycle": 0.0,
        "vis_points": {
            "blind_spot_pts": [],
            "normal_lane_pts": [],
            "crossroad_y": None,
        },
    }


def get_last_valid_state():
    """丢帧时返回上一帧有效值"""
    global _LAST_VALID_STATE, _CONSECUTIVE_DROP_COUNT
    if _LAST_VALID_STATE is not None:
        state = dict(_LAST_VALID_STATE)
        state["frame_dropped"] = True
        state["drop_reason"] = "using_last_valid"
        state["crossroad_detected"] = False
        if _CONSECUTIVE_DROP_COUNT > _MAX_CONSECUTIVE_DROPS:
            state["drop_reason"] = "visual_fault_fallback"
        return state
    return _empty_lane_state(frame_dropped=True, drop_reason="no_history")


def analyze_bev_lane_state(
    bev_mask: np.ndarray,
    ipm_engine,  # MathematicalIPM 实例
    blind_spot_px: int,
    track_width_mm: float,
    window_h: int = 20,
    anchor_min_count: int = 3,
    anchor_max_count: int = 5,
) -> dict:
    """
    三段式滑动窗口状态机。

    阶段1: 提取纯净近端锚点 → 质量门控 → 拟合偏转角
    阶段2: 向下外推物理盲区
    阶段3: 向上侦测 + 路口确认
    """
    global _CONSECUTIVE_DROP_COUNT, _LAST_VALID_STATE

    if bev_mask is None or bev_mask.size == 0:
        return _empty_lane_state()

    bev_h, bev_w = bev_mask.shape[:2]
    ppm = ipm_engine.pixel_per_mm
    new_canvas_h = ipm_engine.new_canvas_h
    canvas_w = ipm_engine.canvas_size[0]
    scan_bottom = new_canvas_h

    def _scan_window(y_top: int, y_bottom: int):
        if y_top < 0: y_top = 0
        if y_bottom > bev_h: y_bottom = bev_h
        if y_bottom <= y_top: return None
        window = bev_mask[y_top:y_bottom, :]
        white_indices = np.where(window == 255)[1]
        if white_indices.size == 0: return None
        return (
            float(np.mean(white_indices)),        # x_center
            float(int(white_indices.max()) - int(white_indices.min())),  # width_px
            int(white_indices.min()),             # left_x
            int(white_indices.max()),             # right_x
        )

    # ================================================================
    # 阶段 1: 提取纯净近端锚点
    # ================================================================
    anchor_pts = []
    last_valid_y_top = scan_bottom
    consecutive_empty = 0
    max_consecutive_empty = 3
    y_bottom = scan_bottom
    first_valid_found = False

    while y_bottom > 0:
        y_top = max(0, y_bottom - window_h)
        result = _scan_window(y_top, y_bottom)
        if result is None:
            consecutive_empty += 1
            if consecutive_empty > max_consecutive_empty: break
            y_bottom -= window_h
            continue

        if not first_valid_found:
            first_valid_found = True
            consecutive_empty = 0
            x_center, width_px, left_x, right_x = result
            y_mid = (y_top + y_bottom) / 2.0
            anchor_pts.append((y_mid, x_center, left_x, right_x, width_px))
            last_valid_y_top = y_top
            y_bottom -= window_h
            continue

        consecutive_empty = 0
        x_center, width_px, left_x, right_x = result
        y_mid = (y_top + y_bottom) / 2.0
        anchor_pts.append((y_mid, x_center, left_x, right_x, width_px))
        last_valid_y_top = y_top
        y_bottom -= window_h

        if len(anchor_pts) >= anchor_max_count: break

    if len(anchor_pts) < anchor_min_count:
        _CONSECUTIVE_DROP_COUNT += 1
        return _empty_lane_state(frame_dropped=True, drop_reason="too_few_anchors")

    # --- 质量门控 ---
    quality_result = validate_frame_quality(anchor_pts, ppm, track_width_mm)
    if not quality_result["valid"]:
        _CONSECUTIVE_DROP_COUNT += 1
        return _empty_lane_state(
            quality_score=quality_result["quality_score"],
            frame_dropped=True,
            drop_reason=quality_result["reason"],
        )
    _CONSECUTIVE_DROP_COUNT = 0

    # --- 拟合 + 偏转角 ---
    ys = np.array([p[0] for p in anchor_pts], dtype=np.float64)
    xs = np.array([p[1] for p in anchor_pts], dtype=np.float64)
    coeffs = np.polyfit(ys, xs, 1)
    a, b_coeff = coeffs[0], coeffs[1]
    lane_angle_rad = math.atan(a)

    # ================================================================
    # 阶段 2: 向下外推物理盲区
    # ================================================================
    blind_spot_pts = []
    anchor_start_y = int(anchor_pts[0][0])
    step = 5
    for y in range(anchor_start_y, new_canvas_h + 1, step):
        x = a * y + b_coeff
        blind_spot_pts.append((float(x), int(y)))

    if not blind_spot_pts or blind_spot_pts[-1][1] != new_canvas_h:
        x_bottom = a * new_canvas_h + b_coeff
        blind_spot_pts.append((float(x_bottom), new_canvas_h))
        target_x_bottom = float(x_bottom)
    else:
        target_x_bottom = blind_spot_pts[-1][0]

    # ================================================================
    # 阶段 3: 向上侦测 + 路口确认
    # ================================================================
    normal_lane_pts = [(float(p[1]), int(p[0])) for p in anchor_pts]
    crossroad_detected = False
    y_crossroad = None
    duty_cycle = 0.0

    y_bottom = last_valid_y_top
    while y_bottom > 0:
        y_top = max(0, y_bottom - window_h)
        result = _scan_window(y_top, y_bottom)
        if result is None:
            y_bottom -= window_h
            continue

        x_center, width_px, left_x, right_x = result
        is_cr, dc = detect_crossroad(bev_mask, y_top, y_bottom, canvas_w)
        if is_cr:
            duty_cycle = dc
            y_crossroad = y_top
            set_last_duty(dc)
            crossroad_detected, _ = confirm_crossroad(True)
            break
        else:
            confirm_crossroad(False)

        y_mid = (y_top + y_bottom) / 2.0
        normal_lane_pts.append((float(x_center), int(y_mid)))
        y_bottom -= window_h

    # 横向误差
    pid_error_mm = (target_x_bottom - (canvas_w / 2.0)) / ppm

    # 路口距离
    distance_to_crossroad_mm = -1.0
    if crossroad_detected and y_crossroad is not None:
        distance_to_crossroad_mm = (new_canvas_h - y_crossroad) / ppm

    lane_state = {
        "pid_error_mm": pid_error_mm,
        "crossroad_detected": crossroad_detected,
        "distance_to_crossroad_mm": distance_to_crossroad_mm,
        "lane_angle_rad": lane_angle_rad,
        "quality_score": quality_result["quality_score"],
        "frame_dropped": False,
        "drop_reason": "",
        "duty_cycle": duty_cycle,
        "vis_points": {
            "blind_spot_pts": blind_spot_pts,
            "normal_lane_pts": normal_lane_pts,
            "crossroad_y": y_crossroad,
        },
    }

    _LAST_VALID_STATE = lane_state
    return lane_state
