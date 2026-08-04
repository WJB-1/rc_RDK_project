# -*- coding: utf-8 -*-
"""
帧质量门控 (Frame Quality Gate)

基于 IPM 物理尺度检验单帧分割质量：
  - 近端窗口宽度校验 (0.6× ~ 1.4× track_width)
  - 边缘光滑性检查 (相邻窗口跳变 < 0.4× track_width_px)
  - 方向一致性检查 (左右边缘应向外扩符号相反, 毛边反之)
"""


def validate_frame_quality(anchor_pts, ppm, track_width_mm):
    """
    只看近端锚点的帧质量门控。远端可能是路口，不做质量判定。

    anchor_pts: [(y_mid, x_center, left_x, right_x, width_px), ...]
    ppm: 像素/毫米
    track_width_mm: 标定车道物理宽度

    条件（全部通过才为 valid）:
    1. 锚点数量 ≥ 3
    2. 近端第1窗口宽度 in [0.6×, 1.4×] track_width
    3. 近端边缘跳变 < 0.4× track_width_px
    4. 左右边缘跳变方向一致（毛边: 同号且大跳变）

    Returns: {"valid": bool, "quality_score": float 0~1, "reason": str}
    """
    n = len(anchor_pts)
    if n < 3:
        return {"valid": False, "quality_score": 0.0, "reason": "too_few_anchors"}

    track_width_px = track_width_mm * ppm

    # --- 近端第1窗口宽度 ---
    near_width_px = anchor_pts[0][4]
    near_width_mm = near_width_px / ppm
    if near_width_mm < 0.6 * track_width_mm or near_width_mm > 1.4 * track_width_mm:
        return {"valid": False, "quality_score": 0.0, "reason": "near_width_out_of_range"}

    # --- 近端前3个锚点边缘光滑性 ---
    near_anchors = anchor_pts[:min(3, n)]
    left_edges = [a[2] for a in near_anchors]
    right_edges = [a[3] for a in near_anchors]

    left_deltas = [left_edges[i + 1] - left_edges[i] for i in range(len(left_edges) - 1)]
    right_deltas = [right_edges[i + 1] - right_edges[i] for i in range(len(right_edges) - 1)]

    max_jump_px = max([abs(d) for d in left_deltas + right_deltas] + [0])
    if max_jump_px > 0.4 * track_width_px:
        return {"valid": False, "quality_score": 0.0, "reason": "edge_jagged"}

    # 方向一致性: 正常车道左右边缘往外扩, 符号相反 (dl<=0, dr>=0)
    # 毛边: 同号且跳变 > 0.15×车道宽
    for dl, dr in zip(left_deltas, right_deltas):
        if dl * dr > 0 and abs(dl) > 0.15 * track_width_px and abs(dr) > 0.15 * track_width_px:
            return {"valid": False, "quality_score": 0.0, "reason": "lane_drift"}

    # --- 质量评分 ---
    near_deviation = abs(near_width_mm - track_width_mm) / track_width_mm
    edge_roughness = max_jump_px / track_width_px if track_width_px > 0 else 0
    quality_score = max(0.0, 1.0 - 0.6 * near_deviation - 0.4 * edge_roughness)

    return {"valid": True, "quality_score": quality_score, "reason": "ok"}
