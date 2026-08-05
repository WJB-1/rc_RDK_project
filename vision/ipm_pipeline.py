# -*- coding: utf-8 -*-
"""
向后兼容层 — 已迁移到 vision/algorithms/ + perception/ipm_drawer.py

如需新代码，请直接 import:
  from vision.algorithms.ipm import MathematicalIPM
  from vision.algorithms.lane_analyzer import analyze_bev_lane_state, get_last_valid_state
  from vision.algorithms.quality_gate import validate_frame_quality
  from vision.algorithms.crossroad_detect import detect_crossroad, reset_crossroad_counter
  from vision.algorithms.mask_utils import clean_mask_by_cc
  from perception.ipm_drawer import draw_debug_panel
"""

from vision.algorithms.ipm import MathematicalIPM
from vision.algorithms.lane_analyzer import (
    analyze_bev_lane_state, get_last_valid_state,
    reset_drop_counter, _empty_lane_state,
)
from vision.algorithms.quality_gate import validate_frame_quality
from vision.algorithms.crossroad_detect import (
    detect_crossroad, confirm_crossroad, reset_crossroad_counter,
    set_last_duty, get_last_duty,
)
from vision.algorithms.mask_utils import clean_mask_by_cc

# 保留兼容别名
from vision.algorithms.lane_analyzer import _empty_lane_state as empty_lane_state
_validate_frame_quality = validate_frame_quality
_detect_crossroad_duty_cycle = detect_crossroad
from perception.ipm_drawer import draw_debug_panel as draw_debug_panel_math_ipm


def run_math_ipm_pipeline(mask, ipm_engine, raw_image=None, physical_track_width_mm=450.0):
    """兼容旧调用方的便捷封装"""
    h, w = mask.shape[:2]
    clean_mask, noise_mask = clean_mask_by_cc(mask, min_bottom_y=h - 10)
    bev_mask = ipm_engine.warp(clean_mask)

    lane_state = analyze_bev_lane_state(
        bev_mask=bev_mask, ipm_engine=ipm_engine,
        blind_spot_px=ipm_engine.blind_spot_px,
        track_width_mm=physical_track_width_mm,
    )
    if lane_state.get("frame_dropped", False):
        lane_state = get_last_valid_state()

    debug_panel = draw_debug_panel(
        clean_mask=clean_mask, bev_mask=bev_mask, lane_state=lane_state,
        raw_image=raw_image, camera_pitch_deg=ipm_engine.pitch_deg,
        physical_track_width_mm=physical_track_width_mm, noise_mask=noise_mask,
    )

    return {
        "clean_mask": clean_mask, "noise_mask": noise_mask,
        "bev_mask": bev_mask, "lane_state": lane_state,
        "debug_panel": debug_panel,
    }
