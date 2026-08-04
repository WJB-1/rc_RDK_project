# -*- coding: utf-8 -*-
"""
IPM Debug 可视化绘制 (编排层)

将 vision 算法输出的 lane_state 渲染为四宫格调试面板。
纯绘制逻辑，不涉及算法计算。
"""

import math
import numpy as np
import cv2


def draw_debug_panel(
    clean_mask: np.ndarray,
    bev_mask: np.ndarray,
    lane_state: dict,
    raw_image: np.ndarray = None,
    camera_pitch_deg: float = 40.0,
    physical_track_width_mm: float = 200.0,
    noise_mask: np.ndarray = None,
) -> np.ndarray:
    """
    四宫格 Debug 面板:
      左上: Clean Mask — 绿色=(保留赛道) 红色=(被清除噪点) 紫色轮廓=(噪点边界)
      右上: Mathematical BEV — 鸟瞰图
      左下: Lane State — 白=车道线 绿=盲区预测 蓝=路口截断 红竖线=车头中心
      右下: Telemetry — 实时数据
    """
    cell_h, cell_w = 400, 400
    bev_h, bev_w = bev_mask.shape[:2]

    # ---------- 左上: Clean Mask ----------
    if raw_image is not None:
        vis_clean = cv2.resize(raw_image, (cell_w, cell_h))
        if noise_mask is not None and np.any(noise_mask):
            noise_small = cv2.resize(noise_mask, (cell_w, cell_h), interpolation=cv2.INTER_NEAREST)
            red_overlay = np.zeros_like(vis_clean)
            red_overlay[noise_small == 255] = (0, 0, 255)
            vis_clean = cv2.addWeighted(vis_clean, 0.65, red_overlay, 0.6, 0)
        clean_small = cv2.resize(clean_mask, (cell_w, cell_h), interpolation=cv2.INTER_NEAREST)
        green_overlay = np.zeros_like(vis_clean)
        green_overlay[clean_small == 255] = (0, 255, 0)
        vis_clean = cv2.addWeighted(vis_clean, 1.0, green_overlay, 0.4, 0)
    else:
        vis_clean = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
        clean_small = cv2.resize(clean_mask, (cell_w, cell_h), interpolation=cv2.INTER_NEAREST)
        if noise_mask is not None and np.any(noise_mask):
            noise_small = cv2.resize(noise_mask, (cell_w, cell_h), interpolation=cv2.INTER_NEAREST)
            vis_clean[noise_small == 255] = (0, 0, 255)
        vis_clean[clean_small == 255] = (0, 255, 0)

    noise_contour_count = 0
    if noise_mask is not None and np.any(noise_mask):
        noise_small = cv2.resize(noise_mask, (cell_w, cell_h), interpolation=cv2.INTER_NEAREST)
        noise_contours, _ = cv2.findContours(noise_small, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis_clean, noise_contours, -1, (255, 0, 255), 1)
        noise_contour_count = len(noise_contours)

    cv2.putText(vis_clean, "Clean Mask", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # ---------- 右上: BEV ----------
    vis_bev = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    vis_bev = cv2.resize(vis_bev, (cell_w, cell_h))
    cv2.putText(vis_bev, "Mathematical BEV", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # ---------- 左下: Lane State ----------
    vis_sw = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    vis_points = lane_state.get("vis_points", {})
    blind_spot_pts = vis_points.get("blind_spot_pts", [])
    normal_lane_pts = vis_points.get("normal_lane_pts", [])
    crossroad_y = vis_points.get("crossroad_y", None)
    crossroad_detected = lane_state.get("crossroad_detected", False)

    if len(normal_lane_pts) > 1:
        pts_arr = np.array([(int(x), int(y)) for x, y in normal_lane_pts], dtype=np.int32)
        cv2.polylines(vis_sw, [pts_arr], False, (255, 255, 255), 2)
    for x, y in normal_lane_pts:
        cv2.circle(vis_sw, (int(x), int(y)), 2, (255, 255, 255), -1)

    if len(blind_spot_pts) > 1:
        pts_arr = np.array([(int(x), int(y)) for x, y in blind_spot_pts], dtype=np.int32)
        cv2.polylines(vis_sw, [pts_arr], False, (0, 255, 0), 2)
    for x, y in blind_spot_pts:
        cv2.circle(vis_sw, (int(x), int(y)), 2, (0, 255, 0), -1)

    if crossroad_detected and crossroad_y is not None:
        cv2.line(vis_sw, (0, crossroad_y), (bev_w - 1, crossroad_y), (255, 0, 0), 2)
        if crossroad_y > 0:
            overlay = vis_sw.copy()
            overlay[:crossroad_y, :] = (255, 0, 0)
            vis_sw = cv2.addWeighted(vis_sw, 0.6, overlay, 0.4, 0)

    canvas_center_x = bev_w // 2
    cv2.line(vis_sw, (canvas_center_x, 0), (canvas_center_x, bev_h - 1), (0, 0, 255), 1)

    vis_sw = cv2.resize(vis_sw, (cell_w, cell_h))
    cv2.putText(vis_sw, "Lane State (Physical)", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # ---------- 右下: Telemetry ----------
    pid_error_mm = lane_state.get("pid_error_mm", 0.0)
    distance_to_crossroad_mm = lane_state.get("distance_to_crossroad_mm", -1.0)
    quality_score = lane_state.get("quality_score", -1.0)
    frame_dropped = lane_state.get("frame_dropped", False)
    lane_angle_rad = lane_state.get("lane_angle_rad", 0.0)
    duty_cycle = lane_state.get("duty_cycle", 0.0)

    vis_text = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)
    lines = [
        "===== Math IPM Telemetry =====", "",
        f"Pitch: {camera_pitch_deg:.1f} deg | Track: {physical_track_width_mm:.0f} mm",
        f"PID Error: {pid_error_mm:+.1f} mm",
        f"Crossroad: {crossroad_detected}",
        f"Quality: {quality_score:.2f}",
        f"Lane Angle: {math.degrees(lane_angle_rad):.1f} deg",
        f"Duty Cycle: {duty_cycle:.2f}",
    ]
    if crossroad_detected:
        lines.append(f"Dist to Cross: {distance_to_crossroad_mm:.0f} mm")
    if frame_dropped:
        lines.append(f"DROP: {lane_state.get('drop_reason', '')}")
    lines.append(f"Normal pts: {len(normal_lane_pts)} | Blind pts: {len(blind_spot_pts)}")
    if noise_mask is not None and np.any(noise_mask):
        noise_px = np.count_nonzero(noise_mask)
        lines.append(f"Noise CC: {noise_contour_count} | px: {noise_px}")

    y_offset = 40
    for line in lines:
        cv2.putText(vis_text, line, (20, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        y_offset += 28

    top_row = np.hstack([vis_clean, vis_bev])
    bottom_row = np.hstack([vis_sw, vis_text])
    return np.vstack([top_row, bottom_row])
