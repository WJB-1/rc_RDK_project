# -*- coding: utf-8 -*-
"""Lane detection debug visualizations.

Standalone drawing helpers used by LaneTracker and the web preview.
All functions take plain numpy arrays / dicts and return BGR images.
Functions return None when the required input is missing, so callers can
safely skip stages that the current pipeline did not produce.
"""
from __future__ import annotations

import math

import cv2
import numpy as np


LINE_COLORS = {
    "0": (0, 200, 255), "1": (0, 100, 255),
    "2": (0, 255, 0),   "3": (0, 180, 0),
    "4": (255, 0, 255), "5": (180, 0, 180),
}
ALL_IDS = ("0", "1", "2", "3", "4", "5")
LANE_IDS = ("0", "1")


class BevCanvas:
    """Minimal BEV canvas used for debug drawing.

    Kept independent from perception.algorithms.ipm so this module does not
    pull in heavy dependencies just to render a panel.
    """

    def __init__(self, x_min=-600.0, x_max=600.0,
                 y_min=-250.0, y_max=800.0, ppm=0.9):
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.ppm = ppm
        self.width = int(round((x_max - x_min) * ppm))
        self.height = int(round((y_max - y_min) * ppm))

    def to_px(self, x, y):
        u = (x - self.x_min) * self.ppm
        v = self.height - 1 - (y - self.y_min) * self.ppm
        return int(round(u)), int(round(v))


# ---------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------
def _ensure_bgr(img):
    if img is None:
        return None
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def _to_u8(img):
    if img is None:
        return None
    if img.dtype == np.uint8:
        return img
    return np.clip(img * 255.0, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------
# Stage views
# ---------------------------------------------------------------------
def vis_enhanced(enhanced):
    return _ensure_bgr(enhanced)


def vis_edge_prob(edge_prob):
    if edge_prob is None:
        return None
    return cv2.applyColorMap(_to_u8(edge_prob), cv2.COLORMAP_INFERNO)


def vis_binary(binary):
    return _ensure_bgr(binary)


def vis_closed(closed):
    return _ensure_bgr(closed)


def vis_labels(label_map, n_labels):
    if label_map is None:
        return None
    if n_labels <= 1:
        return np.zeros((*label_map.shape, 3), dtype=np.uint8)
    rng = np.random.default_rng(42)
    colors = np.zeros((n_labels, 3), dtype=np.uint8)
    colors[1:] = rng.integers(60, 255, size=(n_labels - 1, 3), dtype=np.uint8)
    return colors[label_map]


def vis_skeleton(skeleton):
    return _ensure_bgr(skeleton)


def vis_hough(enhanced, hough_lines):
    if enhanced is None:
        return None
    out = _ensure_bgr(enhanced).copy()
    lines = list(hough_lines or [])
    for line in lines:
        p1 = (int(round(line["x1"])), int(round(line["y1"])))
        p2 = (int(round(line["x2"])), int(round(line["y2"])))
        cv2.line(out, p1, p2, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(out, f"Hough: {len(lines)}", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------
# BEV views
# ---------------------------------------------------------------------
def draw_bev_grid(bev, canvas):
    for x in range(int(canvas.x_min), int(canvas.x_max) + 1, 100):
        u, _ = canvas.to_px(x, 0)
        if 0 <= u < canvas.width:
            c = (80, 80, 80) if x % 200 == 0 else (40, 40, 40)
            cv2.line(bev, (u, 0), (u, canvas.height - 1), c, 1, cv2.LINE_AA)
    for y in range(int(canvas.y_min), int(canvas.y_max) + 1, 100):
        _, v = canvas.to_px(0, y)
        if 0 <= v < canvas.height:
            c = (80, 80, 80) if y % 200 == 0 else (40, 40, 40)
            cv2.line(bev, (0, v), (canvas.width - 1, v), c, 1, cv2.LINE_AA)


def _blank_bev(canvas):
    bev = np.full((canvas.height, canvas.width, 3), 20, dtype=np.uint8)
    draw_bev_grid(bev, canvas)
    return bev


def vis_ground_segments_bev(canvas, ground_segs):
    bev = _blank_bev(canvas)
    segs = list(ground_segs or [])
    for s in segs:
        g1, g2 = s.get("g1"), s.get("g2")
        if g1 is None or g2 is None:
            continue
        cv2.line(bev, canvas.to_px(*g1), canvas.to_px(*g2),
                 (140, 140, 140), 2, cv2.LINE_AA)
        mid = canvas.to_px(s.get("mid_x", 0.0), s.get("mid_y", 0.0))
        cv2.putText(bev, f"{s.get('length_mm', 0.0):.0f}",
                    (mid[0] + 2, mid[1] - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(bev, f"ground segments: {len(segs)}", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return bev


def vis_merged_bev(canvas, merged):
    bev = _blank_bev(canvas)
    items = list(merged or [])
    for m in items:
        p1, p2 = m.get("p1"), m.get("p2")
        if p1 is None or p2 is None:
            continue
        cv2.line(bev, canvas.to_px(*p1), canvas.to_px(*p2),
                 (200, 200, 200), 3, cv2.LINE_AA)
        mid = canvas.to_px(m.get("mid_x", 0.0), m.get("mid_y", 0.0))
        cv2.putText(
            bev,
            f"n={m.get('member_count', 0)} L={m.get('length_mm', 0.0):.0f}",
            (mid[0] + 3, mid[1]),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4,
            (255, 255, 255), 1, cv2.LINE_AA,
        )
    cv2.putText(bev, f"merged: {len(items)}", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return bev


def vis_candidates_bev(canvas, merged, slots, template, radius_mm):
    bev = _blank_bev(canvas)
    for m in merged or []:
        p1, p2 = m.get("p1"), m.get("p2")
        if p1 is None or p2 is None:
            continue
        cv2.line(bev, canvas.to_px(*p1), canvas.to_px(*p2),
                 (90, 90, 90), 2, cv2.LINE_AA)

    template = template or {}
    slots = slots or {}
    for lid in ALL_IDS:
        e = float(template.get(lid, 0.0))
        color = LINE_COLORS[lid]
        p_lo = canvas.to_px(e - radius_mm, canvas.y_min)
        p_bot = canvas.to_px(e + radius_mm, canvas.y_max)
        overlay = bev.copy()
        cv2.rectangle(overlay, p_lo, p_bot, color, -1)
        cv2.addWeighted(overlay, 0.12, bev, 0.88, 0, bev)
        pt1 = canvas.to_px(e, canvas.y_min)
        pt2 = canvas.to_px(e, canvas.y_max)
        cv2.line(bev, pt1, pt2, color, 1, cv2.LINE_AA)
        n = len(slots.get(lid, []) or [])
        cv2.putText(bev, f"{lid}(n={n})",
                    (pt1[0] + 3, 30 + 16 * int(lid)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    cv2.putText(bev, f"candidates R={radius_mm}mm", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return bev


def _ground_line_endpoints(x_at_ref, theta, y_ref, y_low, y_high):
    t = math.tan(theta)
    return ((x_at_ref + (y_low - y_ref) * t, y_low),
            (x_at_ref + (y_high - y_ref) * t, y_high))


def vis_final_bev(canvas, merged, result, y_ref):
    bev = _blank_bev(canvas)
    for m in merged or []:
        p1, p2 = m.get("p1"), m.get("p2")
        if p1 is None or p2 is None:
            continue
        cv2.line(bev, canvas.to_px(*p1), canvas.to_px(*p2),
                 (90, 90, 90), 1, cv2.LINE_AA)

    lines = (result or {}).get("lines") or {}
    for lid in ALL_IDS:
        info = lines.get(lid)
        if info is None:
            continue
        color = LINE_COLORS[lid]
        g1, g2 = _ground_line_endpoints(
            info.get("x_at_ref_mm", 0.0), info.get("theta", 0.0),
            y_ref, canvas.y_min, canvas.y_max,
        )
        p1 = canvas.to_px(*g1)
        p2 = canvas.to_px(*g2)
        thickness = 3 if info.get("matched") else 1
        cv2.line(bev, p1, p2, color, thickness, cv2.LINE_AA)
        mid = canvas.to_px(info.get("x_at_ref_mm", 0.0), y_ref)
        tag = lid if info.get("matched") else lid + "*"
        cv2.putText(bev, tag, (mid[0] + 4, mid[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return bev


def vis_final_overlay(image, ground_to_pixel, result,
                      scale_up_x, scale_up_y, y_ref):
    if image is None:
        return None
    out = image.copy()
    lines = (result or {}).get("lines") or {}
    if not lines:
        return out
    for lid in ALL_IDS:
        info = lines.get(lid)
        if info is None:
            continue
        color = LINE_COLORS[lid]
        g1, g2 = _ground_line_endpoints(
            info.get("x_at_ref_mm", 0.0), info.get("theta", 0.0),
            y_ref, 0.0, 800.0,
        )
        p1 = ground_to_pixel(g1)
        p2 = ground_to_pixel(g2)
        if p1 is None or p2 is None:
            continue
        p1 = (round(p1[0] * scale_up_x), round(p1[1] * scale_up_y))
        p2 = (round(p2[0] * scale_up_x), round(p2[1] * scale_up_y))
        thickness = 3 if info.get("matched") else 1
        cv2.line(out, p1, p2, color, thickness, cv2.LINE_AA)
        cv2.putText(out, lid, (p1[0] + 5, p1[1] + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------
# New-ground views（由 pidinet_lane.py 的 selector 直接渲染）
# ---------------------------------------------------------------------
def vis_new_ground_bev(capture):
    """BEV + 中心线（由 TemplateDistanceLaneSelector.render_new_ground_bev 生成）。

    图里包含：候选段（蓝细）、六条身份线（实/虚）、图像空间中心线（品红）、
    x=0 参考、车辆示意。属于纯展示，不影响任何控制量。
    """
    img = capture.get("new_ground_bev")
    return _ensure_bgr(img)


def vis_new_ground_original(capture):
    """原图 + 匹配到的 left/right Hough 线 + 图像空间中心线。

    由 TemplateDistanceLaneSelector.render_original_with_ground_centerline 生成。
    可以直接对比原图上看到的车道线斜不斜、中心线在什么位置。
    """
    img = capture.get("new_ground_original")
    return _ensure_bgr(img)


# ---------------------------------------------------------------------
# Registry: name -> callable(captured dict) -> BGR image
# ---------------------------------------------------------------------
VIEW_BUILDERS = {
    "debug_enhanced":    lambda c: vis_enhanced(c.get("enhanced")),
    "debug_edge_prob":   lambda c: vis_edge_prob(c.get("edge_prob")),
    "debug_binary":      lambda c: vis_binary(c.get("binary")),
    "debug_closed":      lambda c: vis_closed(c.get("closed")),
    "debug_labels":      lambda c: vis_labels(
        c.get("labels"), int(c.get("n_labels", 0))),
    "debug_skeleton":    lambda c: vis_skeleton(c.get("skeleton")),
    "debug_hough":       lambda c: vis_hough(
        c.get("enhanced"), c.get("hough_lines")),
    "debug_ground_segments_bev": lambda c: vis_ground_segments_bev(
        c.get("canvas"), c.get("ground_segments")),
    "debug_merged_bev":  lambda c: vis_merged_bev(
        c.get("canvas"), c.get("merged")),
    "debug_candidates_bev": lambda c: vis_candidates_bev(
        c.get("canvas"), c.get("merged"), c.get("candidates"),
        c.get("template"), float(c.get("candidate_radius_mm", 40.0))),
    "debug_final_bev":   lambda c: vis_final_bev(
        c.get("canvas"), c.get("merged"), c.get("result"),
        float(c.get("y_ref_mm", 500.0))),
    "debug_final_overlay": lambda c: vis_final_overlay(
        c.get("original_input"), c.get("ground_to_pixel"),
        c.get("result"),
        float(c.get("scale_up_x", 1.0)),
        float(c.get("scale_up_y", 1.0)),
        float(c.get("y_ref_mm", 500.0))),

    # 新地面系可视化
    "new_ground_bev":      vis_new_ground_bev,
    "new_ground_original": vis_new_ground_original,
}