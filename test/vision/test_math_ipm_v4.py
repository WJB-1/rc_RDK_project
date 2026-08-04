#!/usr/bin/env python3
"""
测试 math_ipm_pipeline.py V4.0 新增功能:
  - 帧质量门控 (_validate_frame_quality)
  - 路口占空比检测 (_detect_crossroad_duty_cycle)
  - 锚点数据结构扩展
  - 视觉偏转角提取
  - 丢帧兜底 (get_last_valid_state)
  - 连续帧路口确认
"""

import sys
import math
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from robocup_rescue_brain.vision.algorithms.ipm import MathematicalIPM
from robocup_rescue_brain.vision.algorithms.lane_analyzer import (
    analyze_bev_lane_state, get_last_valid_state,
    reset_drop_counter, _empty_lane_state,
)
from robocup_rescue_brain.vision.algorithms.quality_gate import validate_frame_quality as _validate_frame_quality
from robocup_rescue_brain.vision.algorithms.crossroad_detect import (
    detect_crossroad as _detect_crossroad_duty_cycle, reset_crossroad_counter
)
from robocup_rescue_brain.vision.algorithms.mask_utils import clean_mask_by_cc
from robocup_rescue_brain.vision.ipm_pipeline import run_math_ipm_pipeline

PASS = 0
FAIL = 0


def check(name, condition, detail=""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name} -- {detail}")


# ================================================================
# Test 1: _empty_lane_state
# ================================================================
def test_empty_lane_state():
    print("\n--- Test: _empty_lane_state ---")
    s = _empty_lane_state()
    check("returns dict", isinstance(s, dict))
    check("has pid_error_mm", "pid_error_mm" in s)
    check("has crossroad_detected", "crossroad_detected" in s)
    check("has lane_angle_rad", "lane_angle_rad" in s)
    check("has quality_score", "quality_score" in s)
    check("has frame_dropped", "frame_dropped" in s)
    check("has duty_cycle", "duty_cycle" in s)
    check("frame_dropped=False by default", not s["frame_dropped"])

    s2 = _empty_lane_state(quality_score=0.7, frame_dropped=True, drop_reason="test")
    check("quality_score passed through", s2["quality_score"] == 0.7)
    check("frame_dropped passed through", s2["frame_dropped"])


# ================================================================
# Test 2: _validate_frame_quality - normal lane
# ================================================================
def test_validate_good_lane():
    print("\n--- Test: _validate_frame_quality (good lane) ---")
    # 模拟一条完美直道: 200mm 宽，左右边缘光滑
    ppm = 0.5  # 1px = 2mm
    track_width_mm = 200.0
    # 窗口宽度约 100px (~200mm)
    w = 100.0
    anchor_pts = [
        (250.0, 300.0, 250.0, 350.0, w),   # 近端
        (230.0, 301.0, 249.0, 351.0, 102.0),
        (210.0, 300.0, 250.0, 350.0, w),
    ]
    r = _validate_frame_quality(anchor_pts, ppm, track_width_mm)
    check("good lane valid", r["valid"], f"reason={r.get('reason')}")
    check("quality_score > 0.7", r["quality_score"] > 0.7, f"score={r['quality_score']:.3f}")


# ================================================================
# Test 3: _validate_frame_quality - too few anchors
# ================================================================
def test_validate_too_few():
    print("\n--- Test: _validate_frame_quality (too few anchors) ---")
    anchor_pts = [(250.0, 300.0, 250.0, 350.0, 100.0)]
    r = _validate_frame_quality(anchor_pts, 0.5, 200.0)
    check("too few invalid", not r["valid"])
    check("reason=too_few_anchors", r["reason"] == "too_few_anchors")


# ================================================================
# Test 4: _validate_frame_quality - near width out of range
# ================================================================
def test_validate_bad_width():
    print("\n--- Test: _validate_frame_quality (bad near width) ---")
    # 近端宽度过窄: 50px = 100mm (0.5x track_width, out of 0.6~1.4x)
    anchor_pts = [
        (250.0, 300.0, 275.0, 325.0, 50.0),   # ~100mm, too narrow
        (230.0, 300.0, 248.0, 352.0, 104.0),
        (210.0, 300.0, 250.0, 350.0, 100.0),
    ]
    r = _validate_frame_quality(anchor_pts, 0.5, 200.0)
    check("bad width invalid", not r["valid"])
    check("reason=near_width_out_of_range", r["reason"] == "near_width_out_of_range")


# ================================================================
# Test 5: _validate_frame_quality - jagged edges
# ================================================================
def test_validate_jagged():
    print("\n--- Test: _validate_frame_quality (jagged edges) ---")
    track_width_mm = 200.0
    ppm = 0.5
    tw_px = track_width_mm * ppm  # 100px
    # 相邻窗口边缘跳变 > 0.4 * 100 = 40px
    anchor_pts = [
        (250.0, 300.0, 250.0, 350.0, 100.0),   # left=250
        (230.0, 300.0, 200.0, 350.0, 150.0),    # left jump = 50
        (210.0, 300.0, 250.0, 350.0, 100.0),
    ]
    r = _validate_frame_quality(anchor_pts, ppm, track_width_mm)
    check("jagged invalid", not r["valid"])
    check("reason=edge_jagged", r["reason"] == "edge_jagged")


# ================================================================
# Test 6: _validate_frame_quality - lane drift (毛边)
# ================================================================
def test_validate_drift():
    print("\n--- Test: _validate_frame_quality (lane drift) ---")
    track_width_mm = 200.0
    ppm = 0.5
    tw_px = track_width_mm * ppm
    # 左右边缘同时左移(同号)，幅度 > 0.15*tw_px
    anchor_pts = [
        (250.0, 300.0, 250.0, 350.0, 100.0),
        (230.0, 320.0, 270.0, 370.0, 100.0),   # left+20, right+20, 同号!
        (210.0, 280.0, 290.0, 390.0, 100.0),   # left+20, right+20
    ]
    r = _validate_frame_quality(anchor_pts, ppm, track_width_mm)
    check("drift invalid", not r["valid"])
    check("reason=lane_drift", r["reason"] == "lane_drift")


# ================================================================
# Test 7: _detect_crossroad_duty_cycle
# ================================================================
def test_duty_cycle():
    print("\n--- Test: _detect_crossroad_duty_cycle ---")
    canvas_w = 600
    h = 20

    # 直道: 水平跨度 ~100px, 占空比 ~16%
    straight = np.zeros((h, canvas_w), dtype=np.uint8)
    straight[:, 250:350] = 255
    is_cr, dc = _detect_crossroad_duty_cycle(straight, 0, h, canvas_w)
    check("straight not crossroad", not is_cr, f"duty={dc:.2f}")
    check("straight duty ~0.16", abs(dc - 0.167) < 0.05, f"duty={dc:.3f}")

    # 路口: 水平跨度 ~550px, 占空比 ~91%
    crossroad = np.zeros((h, canvas_w), dtype=np.uint8)
    crossroad[:, 25:575] = 255
    is_cr2, dc2 = _detect_crossroad_duty_cycle(crossroad, 0, h, canvas_w)
    check("crossroad detected", is_cr2, f"duty={dc2:.2f}")
    check("crossroad duty > 0.8", dc2 > 0.8, f"duty={dc2:.3f}")

    # 毛边/围栏: 水平跨度 ~400px, 占空比 ~67%
    noise = np.zeros((h, canvas_w), dtype=np.uint8)
    noise[:, 100:500] = 255
    is_cr3, dc3 = _detect_crossroad_duty_cycle(noise, 0, h, canvas_w)
    check("noise not crossroad (duty < 0.8)", not is_cr3, f"duty={dc3:.2f}")

    # 空窗口
    empty = np.zeros((h, canvas_w), dtype=np.uint8)
    is_cr4, dc4 = _detect_crossroad_duty_cycle(empty, 0, h, canvas_w)
    check("empty not crossroad", not is_cr4, f"duty={dc4:.2f}")


# ================================================================
# Test 8: analyze_bev_lane_state - full pipeline on synthetic data
# ================================================================
def test_full_pipeline():
    print("\n--- Test: analyze_bev_lane_state (synthetic) ---")
    ipm = MathematicalIPM(
        img_w=1920, img_h=1080,
        focal_length_mm=2.8, pixel_size_mm=0.003,
        camera_height_mm=190.0, pitch_deg=40.0,
        canvas_w=600, canvas_h=800,
        pixel_per_mm=0.5, blind_spot_mm=200.0,
    )

    # 同时重置路口计数器
    reset_crossroad_counter()
    reset_drop_counter()

    # 构造一条干净的直道 mask (200mm 宽)
    # BEV 画布: canvas_w=600, new_canvas_h≈900
    bev = np.zeros((ipm.new_canvas_h, ipm.canvas_size[0]), dtype=np.uint8)
    center_x = 300
    half_w = 50  # 100px ≈ 200mm
    bev[:, center_x - half_w:center_x + half_w] = 255

    state = analyze_bev_lane_state(
        bev_mask=bev,
        ipm_engine=ipm,
        blind_spot_px=ipm.blind_spot_px,
        track_width_mm=200.0,
    )

    check("not frame_dropped", not state["frame_dropped"], f"reason={state.get('drop_reason')}")
    check("not crossroad", not state["crossroad_detected"])
    check("has lane_angle_rad", abs(state["lane_angle_rad"]) < 0.1,
          f"angle={math.degrees(state['lane_angle_rad']):.2f}deg")
    check("quality_score > 0.7", state["quality_score"] > 0.7, f"score={state['quality_score']:.3f}")
    check("pid_error_mm ~0", abs(state["pid_error_mm"]) < 30, f"error={state['pid_error_mm']:.1f}mm")
    check("has vis_points", len(state["vis_points"]["normal_lane_pts"]) > 0)
    check("has blind_spot_pts", len(state["vis_points"]["blind_spot_pts"]) > 0)

    # 路口测试: 构造一个横向满幅的窗口
    reset_crossroad_counter()
    cross_bev = np.zeros((ipm.new_canvas_h, ipm.canvas_size[0]), dtype=np.uint8)
    # 近端正常车道
    near_start = ipm.new_canvas_h - 50
    cross_bev[near_start:, center_x - half_w:center_x + half_w] = 255
    # 远端横向满幅 (路口)
    cross_bev[:near_start - 100, 30:570] = 255

    # 需要跑3帧来触发连续帧确认
    for _ in range(3):
        state_cr = analyze_bev_lane_state(
            bev_mask=cross_bev,
            ipm_engine=ipm,
            blind_spot_px=ipm.blind_spot_px,
            track_width_mm=200.0,
        )

    check("crossroad detected after 3 frames", state_cr["crossroad_detected"])
    check("distance > 0", state_cr["distance_to_crossroad_mm"] > 0,
          f"dist={state_cr['distance_to_crossroad_mm']:.0f}mm")


# ================================================================
# Test 9: get_last_valid_state fallback
# ================================================================
def test_fallback():
    print("\n--- Test: get_last_valid_state ---")
    ipm = MathematicalIPM(
        img_w=1920, img_h=1080,
        camera_height_mm=190.0, pitch_deg=40.0,
        canvas_w=600, canvas_h=800, pixel_per_mm=0.5,
    )
    reset_crossroad_counter()
    reset_drop_counter()

    # 先跑一帧有效帧
    bev = np.zeros((ipm.new_canvas_h, ipm.canvas_size[0]), dtype=np.uint8)
    bev[:, 250:350] = 255
    analyze_bev_lane_state(bev, ipm, ipm.blind_spot_px, 200.0)

    # 再跑一帧空帧
    empty_bev = np.zeros((ipm.new_canvas_h, ipm.canvas_size[0]), dtype=np.uint8)
    state_bad = analyze_bev_lane_state(empty_bev, ipm, ipm.blind_spot_px, 200.0)
    check("empty frame_dropped", state_bad["frame_dropped"])

    # fallback 应该返回上一帧有效值
    fallback = get_last_valid_state()
    check("fallback exists", fallback is not None)
    check("fallback not crossroad", not fallback["crossroad_detected"])


# ================================================================
# Test 10: clean_mask_by_cc noise extraction
# ================================================================
def test_clean_mask():
    print("\n--- Test: clean_mask_by_cc ---")
    mask = np.zeros((400, 600), dtype=np.uint8)
    # 主赛道触底
    mask[300:, 200:400] = 255
    # 孤立噪点不触底
    mask[100:120, 50:70] = 255

    clean, noise = clean_mask_by_cc(mask, min_bottom_y=395)
    check("clean preserved lane", np.any(clean == 255))
    check("noise captured", noise is not None and np.any(noise == 255),
          f"noise_px={np.count_nonzero(noise) if noise is not None else 0}")
    check("clean has no noise", not np.any(clean[100:120, 50:70] == 255))


# ================================================================
# Test 11: duty_cycle crossroad with blocked near (quality gates it)
# ================================================================
def test_gate_before_crossroad():
    print("\n--- Test: quality gate before crossroad ---")
    ipm = MathematicalIPM(
        img_w=1920, img_h=1080,
        camera_height_mm=190.0, pitch_deg=40.0,
        canvas_w=600, canvas_h=800, pixel_per_mm=0.5, blind_spot_mm=200.0,
    )
    reset_crossroad_counter()
    reset_drop_counter()

    # 近端碎裂（宽度异常），远端有横向线
    bev = np.zeros((ipm.new_canvas_h, ipm.canvas_size[0]), dtype=np.uint8)
    # 远端横向满幅
    bev[:100, 30:570] = 255
    # 近端只有碎片
    bev[ipm.new_canvas_h - 50:, 280:290] = 255  # 5px 宽 → ~10mm

    state = analyze_bev_lane_state(bev, ipm, ipm.blind_spot_px, 200.0)
    check("quality gate rejects poor near", state["frame_dropped"],
          f"reason={state.get('drop_reason')}")
    check("crossroad suppressed when near bad", not state["crossroad_detected"])

    # 重置计数器以免污染后续测试
    reset_drop_counter()


# ================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("math_ipm_pipeline.py V4.0 功能测试")
    print("=" * 60)

    test_empty_lane_state()
    test_validate_good_lane()
    test_validate_too_few()
    test_validate_bad_width()
    test_validate_jagged()
    test_validate_drift()
    test_duty_cycle()
    test_full_pipeline()
    test_fallback()
    test_clean_mask()
    test_gate_before_crossroad()

    print(f"\n{'=' * 60}")
    print(f"结果: {PASS} PASS, {FAIL} FAIL")
    print(f"{'=' * 60}")

    if FAIL > 0:
        sys.exit(1)
