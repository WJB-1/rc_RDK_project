# -*- coding: utf-8 -*-

"""
语义分割掩码后处理与数学逆透视架构 (Mathematical IPM Pipeline) — V4.0

核心思想: 信任近端，截断远端，向下外推盲区。
新增: 帧质量门控 + 路口占空比检测 + 视觉偏转角提取

涉及改造:
- 改造一: IPM 画布映射（预留物理盲区，y_offset_mm 强制归零）
- 改造二: 三段式滑动窗口状态机（analyze_bev_lane_state）
- 改造三: 严格解耦接口协议输出
- 改造四: 可视化语义约定（供 video_math_ipm.py 调用）
- 改造五: 帧质量门控（_validate_frame_quality）
- 改造六: 路口占空比检测 + 连续帧确认
- 改造七: 视觉偏转角提取（lane_angle_rad）
"""

import math
import numpy as np
import cv2


# ============================================================================
# 全局状态: 路口连续帧确认计数器 & 丢帧保护
# ============================================================================
_CROSSROAD_COUNTER = 0
_CROSSROAD_CONFIRM_FRAMES = 3
_CROSSROAD_DECAY_FRAMES = 2

_CONSECUTIVE_DROP_COUNT = 0
_MAX_CONSECUTIVE_DROPS = 15  # ~0.5 秒后报警

_LAST_VALID_STATE = None  # 丢帧时兜底


def _empty_lane_state(quality_score=0.0, frame_dropped=False, drop_reason=""):
    """统一构造空/失败状态的返回字典"""
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


def reset_crossroad_counter():
    """重置路口连续帧计数器（转弯完成时调用）"""
    global _CROSSROAD_COUNTER
    _CROSSROAD_COUNTER = 0


def reset_drop_counter():
    """重置丢帧计数器"""
    global _CONSECUTIVE_DROP_COUNT
    _CONSECUTIVE_DROP_COUNT = 0


# ---------------------------------------------------------------------------
# Step 1: 连通域清洗（底层锚定法）
# ---------------------------------------------------------------------------
def clean_mask_by_cc(mask: np.ndarray, min_bottom_y: int):
    """
    1. 接收 BiSeNet 推理出的二值化 mask (255为赛道，0为背景)。
    2. 使用 cv2.connectedComponentsWithStats 获取连通域。
    3. 触底校验: 检查 Bounding Box 底部 (y + h) 是否大于等于 min_bottom_y。
    4. 最大面积: 在满足触底条件的连通域中，筛选 area 最大的一个。
    5. 生成全黑 clean_mask，仅将选中的主连通域填为 255。
    6. 返回 (clean_mask, noise_mask) 元组，noise_mask 为被清除的噪点区域。

    :return: (clean_mask, noise_mask) — 均为 uint8 np.ndarray
    """
    if mask is None or mask.size == 0:
        return mask, None

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )

    if num_labels <= 1:
        return np.zeros_like(mask), None

    candidate_idx = -1
    candidate_area = -1

    for i in range(1, num_labels):
        y = stats[i, cv2.CC_STAT_TOP]
        hh = stats[i, cv2.CC_STAT_HEIGHT]
        bottom = y + hh
        area = stats[i, cv2.CC_STAT_AREA]

        if bottom >= min_bottom_y and area > candidate_area:
            candidate_area = area
            candidate_idx = i

    clean_mask = np.zeros_like(mask)
    if candidate_idx > 0:
        clean_mask[labels == candidate_idx] = 255

    noise_mask = np.zeros_like(mask)
    noise_mask[(mask == 255) & (clean_mask == 0)] = 255

    return clean_mask, noise_mask


# ---------------------------------------------------------------------------
# Step 2: 基于相机内外参的绝对逆透视矩阵构建 (Analytical IPM)
# ---------------------------------------------------------------------------
class MathematicalIPM:
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
        y_offset_mm: float = 0.0,
        blind_spot_mm: float = 0.0,
    ):
        """预计算 M_IPM 矩阵"""
        y_offset_mm = 0.0
        f_y = focal_length_mm / pixel_size_mm

        if blind_spot_mm <= 0.0:
            theta = math.radians(pitch_deg)
            alpha = math.atan(img_h / (2.0 * f_y))
            blind_spot_mm = camera_height_mm / math.tan(theta + alpha)
            blind_spot_mm = max(0.0, blind_spot_mm)

        self.pixel_per_mm = pixel_per_mm
        self.camera_height_mm = camera_height_mm
        self.pitch_deg = pitch_deg
        self.y_offset_mm = y_offset_mm
        self.blind_spot_mm = blind_spot_mm
        self.blind_spot_px = int(blind_spot_mm * pixel_per_mm)
        self.original_canvas_h = canvas_h
        self.new_canvas_h = canvas_h + self.blind_spot_px
        self.canvas_size = (canvas_w, self.new_canvas_h)

        f_x = f_y = focal_length_mm / pixel_size_mm
        c_x = img_w / 2.0
        c_y = img_h / 2.0
        self.K = np.array([
            [f_x, 0, c_x],
            [0, f_y, c_y],
            [0, 0, 1],
        ], dtype=np.float64)

        theta = math.radians(pitch_deg)
        h = camera_height_mm
        E = np.array([
            [1, 0, 0],
            [0, -math.sin(theta), h * math.cos(theta)],
            [0, math.cos(theta), h * math.sin(theta)],
        ], dtype=np.float64)

        H = self.K @ E

        ppm = pixel_per_mm
        M_scale = np.array([
            [ppm, 0, canvas_w / 2.0],
            [0, -ppm, self.new_canvas_h],
            [0, 0, 1],
        ], dtype=np.float64)

        self.M_IPM = M_scale @ np.linalg.inv(H)

    def warp(self, clean_mask: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(
            clean_mask, self.M_IPM, self.canvas_size, flags=cv2.INTER_NEAREST
        )


# ============================================================================
# 帧质量门控 (Phase 1)
# ============================================================================
def _validate_frame_quality(anchor_pts, ppm, track_width_mm):
    """
    基于 IPM 物理尺度的帧质量门控。只看近端锚点。

    判定条件（全部满足才通过）:
    1. 锚点数量 ≥ 3
    2. 近端第1窗口物理宽度 in [0.6×, 1.4×] track_width
    3. 近端边缘光滑 — 相邻窗口 left_x/right_x 跳变 < 0.4× track_width_px
    4. 左右边缘跳变方向一致性 — 正常车道 dl<=0, dr>=0；毛边随机乱跳

    Returns: {"valid": bool, "quality_score": float 0~1, "reason": str}
    """
    n = len(anchor_pts)
    if n < 3:
        return {"valid": False, "quality_score": 0.0, "reason": "too_few_anchors"}

    track_width_px = track_width_mm * ppm

    # --- 条件2: 近端第1窗口宽度校验 ---
    near_width_px = anchor_pts[0][4]
    near_width_mm = near_width_px / ppm
    if near_width_mm < 0.6 * track_width_mm or near_width_mm > 1.4 * track_width_mm:
        return {"valid": False, "quality_score": 0.0, "reason": "near_width_out_of_range"}

    # --- 条件3+4: 近端前3个锚点的边缘光滑性 ---
    near_anchors = anchor_pts[:min(3, n)]
    left_edges = [a[2] for a in near_anchors]
    right_edges = [a[3] for a in near_anchors]

    left_deltas = [left_edges[i+1] - left_edges[i] for i in range(len(left_edges) - 1)]
    right_deltas = [right_edges[i+1] - right_edges[i] for i in range(len(right_edges) - 1)]

    max_jump_px = max([abs(d) for d in left_deltas + right_deltas] + [0])
    if max_jump_px > 0.4 * track_width_px:
        return {"valid": False, "quality_score": 0.0, "reason": "edge_jagged"}

    # 方向一致性: 正常车道左右边缘向外扩符号相反 (dl<=0, dr>=0)
    # 毛边: 同号且跳变 > 0.15×车道宽
    for dl, dr in zip(left_deltas, right_deltas):
        if dl * dr > 0 and abs(dl) > 0.15 * track_width_px and abs(dr) > 0.15 * track_width_px:
            return {"valid": False, "quality_score": 0.0, "reason": "lane_drift"}

    # --- 质量评分 ---
    near_deviation = abs(near_width_mm - track_width_mm) / track_width_mm
    edge_roughness = max_jump_px / track_width_px if track_width_px > 0 else 0
    quality_score = max(0.0, 1.0 - 0.6 * near_deviation - 0.4 * edge_roughness)

    return {"valid": True, "quality_score": quality_score, "reason": "ok"}


# ============================================================================
# 路口检测: 横坐标占空比法 (Phase 2)
# ============================================================================
def _detect_crossroad_duty_cycle(bev_mask, y_top, y_bottom, canvas_w, ppm, track_width_mm):
    """
    基于车道水平跨度占空比的路口检测。
    直道占空比 ≈ 200/1200 = 16%，路口横向车道 ≈ 90%+。
    阈值 80% 可以有效区分毛边围栏（水平跨度通常 < 67%）。
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

    is_crossroad = duty_cycle > 0.8
    return is_crossroad, duty_cycle


# ============================================================================
# 改造二 & 三: 三段式状态机 + 严格解耦接口
# ============================================================================
def analyze_bev_lane_state(
    bev_mask: np.ndarray,
    ipm_engine: MathematicalIPM,
    blind_spot_px: int,
    track_width_mm: float,
    window_h: int = 20,
    anchor_min_count: int = 3,
    anchor_max_count: int = 5,
) -> dict:
    """
    三段式滑动窗口状态机核心函数。

    阶段1: 提取纯净近端锚点 → 帧质量门控 → 视觉偏转角
    阶段2: 向下外推物理盲区
    阶段3: 向上侦测，占空比 > 80% 判定路口

    :return: 严格解耦的控制接口字典
    """
    global _CROSSROAD_COUNTER, _CONSECUTIVE_DROP_COUNT, _LAST_VALID_STATE

    if bev_mask is None or bev_mask.size == 0:
        return _empty_lane_state()

    bev_h, bev_w = bev_mask.shape[:2]
    ppm = ipm_engine.pixel_per_mm
    new_canvas_h = ipm_engine.new_canvas_h
    canvas_w = ipm_engine.canvas_size[0]
    scan_bottom = new_canvas_h

    def scan_window(y_top: int, y_bottom: int):
        if y_top < 0:
            y_top = 0
        if y_bottom > bev_h:
            y_bottom = bev_h
        if y_bottom <= y_top:
            return None
        window = bev_mask[y_top:y_bottom, :]
        white_indices = np.where(window == 255)[1]
        if white_indices.size == 0:
            return None
        x_center = float(np.mean(white_indices))
        left_x = int(white_indices.min())
        right_x = int(white_indices.max())
        width_px = float(right_x - left_x)
        return x_center, width_px, left_x, right_x

    # ================================================================
    # 阶段 1: 提取纯净近端锚点 (Extract Anchors)
    # ================================================================
    anchor_pts = []
    last_valid_y_top = scan_bottom
    consecutive_empty = 0
    max_consecutive_empty = 3

    y_bottom = scan_bottom
    first_valid_found = False
    while y_bottom > 0:
        y_top = max(0, y_bottom - window_h)
        result = scan_window(y_top, y_bottom)
        if result is None:
            consecutive_empty += 1
            if consecutive_empty > max_consecutive_empty:
                break
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

        if len(anchor_pts) >= anchor_max_count:
            break

    if len(anchor_pts) < anchor_min_count:
        _CONSECUTIVE_DROP_COUNT += 1
        return _empty_lane_state(frame_dropped=True, drop_reason="too_few_anchors")

    # --- 帧质量门控 ---
    quality_result = _validate_frame_quality(anchor_pts, ppm, track_width_mm)
    if not quality_result["valid"]:
        _CONSECUTIVE_DROP_COUNT += 1
        return _empty_lane_state(
            quality_score=quality_result["quality_score"],
            frame_dropped=True,
            drop_reason=quality_result["reason"],
        )

    # 质量通过 → 重置丢帧计数
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
    target_x_bottom = None
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
    # 阶段 3: 向上侦测 + 占空比路口检测
    # ================================================================
    normal_lane_pts = [(float(p[1]), int(p[0])) for p in anchor_pts]
    crossroad_detected = False
    y_crossroad = None
    duty_cycle = 0.0

    y_bottom = last_valid_y_top
    while y_bottom > 0:
        y_top = max(0, y_bottom - window_h)
        result = scan_window(y_top, y_bottom)
        if result is None:
            y_bottom -= window_h
            continue

        x_center, width_px, left_x, right_x = result

        # 占空比路口检测（替代原来的 width > 1.3×）
        is_cr, dc = _detect_crossroad_duty_cycle(
            bev_mask, y_top, y_bottom, canvas_w, ppm, track_width_mm
        )
        if is_cr:
            duty_cycle = dc
            y_crossroad = y_top
            # 连续帧确认
            _CROSSROAD_COUNTER += 1
            if _CROSSROAD_COUNTER >= _CROSSROAD_CONFIRM_FRAMES:
                crossroad_detected = True
            break
        else:
            # 衰减但不归零（防止单帧漏检）
            _CROSSROAD_COUNTER = max(0, _CROSSROAD_COUNTER - _CROSSROAD_DECAY_FRAMES)

        y_mid = (y_top + y_bottom) / 2.0
        normal_lane_pts.append((float(x_center), int(y_mid)))
        y_bottom -= window_h

    # PID 横向误差
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


# ============================================================================
# 丢帧兜底: 返回上一帧有效状态
# ============================================================================
def get_last_valid_state():
    """获取上一帧有效状态（丢帧时兜底）"""
    global _LAST_VALID_STATE, _CONSECUTIVE_DROP_COUNT
    if _LAST_VALID_STATE is not None:
        state = dict(_LAST_VALID_STATE)
        state["frame_dropped"] = True
        state["drop_reason"] = "using_last_valid"
        state["crossroad_detected"] = False  # 丢帧时不触发路口
        if _CONSECUTIVE_DROP_COUNT > _MAX_CONSECUTIVE_DROPS:
            state["drop_reason"] = "visual_fault_fallback"
        return state
    return _empty_lane_state(frame_dropped=True, drop_reason="no_history")


# ---------------------------------------------------------------------------
# 改造四: Debug 可视化面板
# ---------------------------------------------------------------------------
def draw_debug_panel_math_ipm(
    clean_mask: np.ndarray,
    bev_mask: np.ndarray,
    lane_state: dict,
    raw_image: np.ndarray = None,
    camera_pitch_deg: float = 40.0,
    physical_track_width_mm: float = 450.0,
    noise_mask: np.ndarray = None,
) -> np.ndarray:
    """绘制新四宫格 Debug 面板"""
    H, W = clean_mask.shape[:2]
    bev_h, bev_w = bev_mask.shape[:2]
    cell_h, cell_w = 400, 400

    # ---------- 左上: Clean Mask ----------
    # 性能优化: 先缩到目标尺寸再画，避免在 1080p 上操作
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

    # ---------- 右上: Mathematical BEV ----------
    vis_bev = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
    vis_bev = cv2.resize(vis_bev, (cell_w, cell_h))
    cv2.putText(vis_bev, "Mathematical BEV", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # ---------- 左下: Lane State Visualization ----------
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
        "===== Math IPM Telemetry =====",
        "",
        f"Camera Pitch: {camera_pitch_deg:.1f} deg",
        f"Track Width: {physical_track_width_mm:.0f} mm",
        f"PID Error: {pid_error_mm:+.1f} mm",
        f"Crossroad: {crossroad_detected}",
        f"Quality: {quality_score:.2f}",
        f"Lane Angle: {math.degrees(lane_angle_rad):.1f} deg",
        f"Duty Cycle: {duty_cycle:.2f}",
    ]
    if crossroad_detected:
        lines.append(f"Dist to Cross: {distance_to_crossroad_mm:.0f} mm")
    if frame_dropped:
        lines.append(f"FRAME DROPPED: {lane_state.get('drop_reason', '')}")
    lines.append(f"Normal pts: {len(normal_lane_pts)}")
    lines.append(f"Blind pts: {len(blind_spot_pts)}")

    if noise_mask is not None and np.any(noise_mask):
        noise_px = np.count_nonzero(noise_mask)
        lines.append(f"Noise CC: {noise_contour_count}  |  px: {noise_px}")

    y_offset = 40
    for line in lines:
        cv2.putText(vis_text, line, (20, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        y_offset += 28

    # ---------- 拼接 ----------
    top_row = np.hstack([vis_clean, vis_bev])
    bottom_row = np.hstack([vis_sw, vis_text])
    panel = np.vstack([top_row, bottom_row])
    return panel


# ---------------------------------------------------------------------------
# 便捷封装: 单帧完整流水线
# ---------------------------------------------------------------------------
def run_math_ipm_pipeline(
    mask: np.ndarray,
    ipm_engine: MathematicalIPM,
    raw_image: np.ndarray = None,
    physical_track_width_mm: float = 450.0,
) -> dict:
    """对单帧 mask 执行完整的 Mathematical IPM Pipeline"""
    import time as _t

    h, w = mask.shape[:2]

    _ta = _t.time()
    clean_mask, noise_mask = clean_mask_by_cc(mask, min_bottom_y=h - 10)
    _t_cc = (_t.time() - _ta) * 1000

    _tb = _t.time()
    bev_mask = ipm_engine.warp(clean_mask)
    _t_warp = (_t.time() - _tb) * 1000

    _tc = _t.time()
    lane_state = analyze_bev_lane_state(
        bev_mask=bev_mask,
        ipm_engine=ipm_engine,
        blind_spot_px=ipm_engine.blind_spot_px,
        track_width_mm=physical_track_width_mm,
    )
    _t_analyze = (_t.time() - _tc) * 1000

    if lane_state.get("frame_dropped", False):
        lane_state = get_last_valid_state()

    _td = _t.time()
    debug_panel = draw_debug_panel_math_ipm(
        clean_mask=clean_mask,
        bev_mask=bev_mask,
        lane_state=lane_state,
        raw_image=raw_image,
        camera_pitch_deg=ipm_engine.pitch_deg,
        physical_track_width_mm=physical_track_width_mm,
        noise_mask=noise_mask,
    )
    _t_draw = (_t.time() - _td) * 1000
    _t_total = (_t.time() - _ta) * 1000

    print(f"[IPM timing] total={_t_total:.0f}ms | "
          f"cc={_t_cc:.0f} warp={_t_warp:.0f} analyze={_t_analyze:.0f} draw={_t_draw:.0f}")

    return {
        "clean_mask": clean_mask,
        "noise_mask": noise_mask,
        "bev_mask": bev_mask,
        "lane_state": lane_state,
        "debug_panel": debug_panel,
    }
