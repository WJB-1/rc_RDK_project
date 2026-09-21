# perception/diagnostics/lane_selector_viz.py
"""把 selector 上的可视化逻辑抽出，selector 只做委托。

renderer 只读取 selector 的状态，不修改。
"""
import math
import cv2
import numpy as np

from perception.algorithms.lane.constants import (
    ALL_LINE_IDS, TEMPLATE_LINE_COLORS,
)


class LaneSelectorRenderer:
    """渲染 selector 上的可视化视图。"""

    def __init__(self, selector):
        self.s = selector

    # ---------------- 基础绘制工具 ----------------
    @staticmethod
    def _draw_segment(canvas, segment, color, thickness):
        first = tuple(np.round(segment[0]).astype(int))
        second = tuple(np.round(segment[1]).astype(int))
        height, width = canvas.shape[:2]
        if not (-500 <= first[0] <= width + 500 and -500 <= first[1] <= height + 500):
            return
        if not (-500 <= second[0] <= width + 500 and -500 <= second[1] <= height + 500):
            return
        cv2.line(canvas, first, second, color, thickness, cv2.LINE_AA)

    @staticmethod
    def _draw_dashed_segment(canvas, segment, color, thickness, dash_px=12):
        first = np.asarray(segment[0], dtype=np.float64)
        second = np.asarray(segment[1], dtype=np.float64)
        direction = second - first
        length = np.linalg.norm(direction)
        if length < 1e-6:
            return
        direction /= length
        for start in np.arange(0.0, length, dash_px * 2.0):
            end = min(start + dash_px, length)
            cv2.line(
                canvas,
                tuple(np.round(first + direction * start).astype(int)),
                tuple(np.round(first + direction * end).astype(int)),
                color, thickness, cv2.LINE_AA,
            )

    def _draw_coordinate_system(self, view):
        height, width = view.shape[:2]
        origin_x, origin_y = width // 2, height - 1
        tick_px = max(1, int(round(10.0 * self.s.pixel_per_mm)))
        for x in range(origin_x, width, tick_px):
            cv2.line(view, (x, 0), (x, height), (42, 42, 42), 1)
        for x in range(origin_x - tick_px, -1, -tick_px):
            cv2.line(view, (x, 0), (x, height), (42, 42, 42), 1)
        for y in range(origin_y, -1, -tick_px):
            cv2.line(view, (0, y), (width, y), (42, 42, 42), 1)
        cv2.arrowedLine(view, (0, origin_y), (width - 4, origin_y), (235, 235, 235), 1,
                        cv2.LINE_AA, tipLength=0.02)
        cv2.arrowedLine(view, (origin_x, origin_y), (origin_x, 4), (235, 235, 235), 1,
                        cv2.LINE_AA, tipLength=0.03)
        cv2.putText(view, "X (mm)", (width - 55, origin_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (235, 235, 235), 1, cv2.LINE_AA)
        cv2.putText(view, "Y fwd (mm)", (origin_x + 6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (235, 235, 235), 1, cv2.LINE_AA)

    def _draw_vehicle(self, view):
        height = view.shape[0]
        s = self.s

        def to_px(x_mm, y_mm):
            return (int(round(s.canvas_w / 2.0 + x_mm * s.pixel_per_mm)),
                    int(round(height - 1 - y_mm * s.pixel_per_mm)))

        body_length = float(s.vehicle_geometry.get("body_length_mm", 142.0))
        body_width = float(s.vehicle_geometry.get("body_width_mm", 120.0))
        camera_forward = float(s.vehicle_geometry.get("camera_forward_of_body_front_mm", 60.0))
        camera_width = float(s.vehicle_geometry.get("camera_width_mm", 31.0))
        camera_length = float(s.vehicle_geometry.get("camera_length_mm", 60.0))
        body_front, body_rear = -camera_forward, -camera_forward - body_length
        cv2.rectangle(view, to_px(-body_width / 2.0, body_front),
                      to_px(body_width / 2.0, body_rear), (0, 190, 255), 2)
        camera_first = to_px(-camera_width / 2.0, 0.0)
        camera_second = to_px(camera_width / 2.0, -camera_length)
        for first, second in (
            (camera_first, (camera_second[0], camera_first[1])),
            ((camera_second[0], camera_first[1]), camera_second),
            (camera_second, (camera_first[0], camera_second[1])),
            ((camera_first[0], camera_second[1]), camera_first),
        ):
            cv2.line(view, first, second, (180, 180, 180), 1, cv2.LINE_AA)

    # ---------------- BEV 视图 ----------------
    def _draw_bev_template(self):
        s = self.s
        view = np.zeros((s.canvas_h, s.canvas_w, 3), dtype=np.uint8)
        self._draw_coordinate_system(view)
        for segment in s.candidate_bev_segments:
            self._draw_segment(view, segment, (200, 80, 0), 1)

        if s.template_identities is not None:
            for lid in ALL_LINE_IDS:
                info = s.template_identities[lid]
                color = TEMPLATE_LINE_COLORS[lid]
                segment = s._identity_bev_segment(info)
                if info["matched"]:
                    self._draw_segment(view, segment, color, 3)
                else:
                    self._draw_dashed_segment(view, segment, color, 1)
                mid_x = int(round(s.canvas_w / 2.0 + info["x_at_ref_mm"] * s.pixel_per_mm))
                mid_y = int(round(s.canvas_h - 1.0 - s.y_ref_mm * s.pixel_per_mm))
                tag = f"{lid}" if info["matched"] else f"{lid}*"
                cv2.putText(view, tag, (mid_x + 4, mid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        self._draw_vehicle(view)
        cv2.circle(view, tuple(np.round(s.vehicle_point).astype(int)), 4, (0, 255, 255), -1)

        return view

    def _draw_bev_ground_ipm(self):
        s = self.s
        view = np.zeros((s.canvas_h, s.canvas_w, 3), dtype=np.uint8)
        self._draw_coordinate_system(view)
        for segment in s.candidate_bev_segments:
            self._draw_segment(view, segment, (200, 80, 0), 1)
        if s.final_pair is not None:
            for segment, color, inferred in (
                (s.final_pair["left"], (0, 255, 0), s.final_pair["left_inferred"]),
                (s.final_pair["right"], (0, 0, 255), s.final_pair["right_inferred"]),
            ):
                if inferred:
                    self._draw_dashed_segment(view, segment, color, 3)
                else:
                    self._draw_segment(view, segment, color, 3)
        self._draw_vehicle(view)
        cv2.circle(view, tuple(np.round(s.vehicle_point).astype(int)), 4, (0, 255, 255), -1)
        if s.final_pair is not None:
            cross_y = int(round(s.final_pair["vehicle_cross_section_y"]))
            centre_x = int(round(s.final_pair["lane_centre_x"]))
            vehicle_x = int(round(s.vehicle_center[0]))
            cv2.line(view, (0, cross_y), (s.canvas_w - 1, cross_y), (0, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(view, (centre_x, cross_y), 4, (255, 255, 0), -1)
            cv2.circle(view, (vehicle_x, cross_y), 4, (0, 255, 255), -1)
        if s.final_pair is None:
            info = f"cand={len(s.candidate_bev_segments)} | pair: none"
        else:
            info = (
                f"cand={len(s.candidate_bev_segments)} | profile={s.final_pair['pair_profile']} "
                f"d={s.final_pair['distance_mm']:.1f}mm "
                f"(|d-{s.final_pair['target_mm']:.0f}|="
                f"{abs(s.final_pair['distance_mm'] - s.final_pair['target_mm']):.1f}) | "
                f"ang={s.final_pair['parallel_angle_deg']:.2f}deg"
            )
        cv2.putText(view, info, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(view, "cand=blue | final: L=green R=red | dashed=inferred", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        return view

    def draw_bev_view(self):
        if hasattr(self.s, "y_ref_mm"):
            return self._draw_bev_template()
        return self._draw_bev_ground_ipm()

    # ---------------- 原图视图 ----------------
    def _draw_original_template(self, processing_frame):
        s = self.s
        view = processing_frame.copy()
        if s.template_identities is not None:
            y_low = 0.0
            y_high = (s.canvas_h - 1.0) / s.pixel_per_mm
            lane_matrix = s._matrix_for_profile("lane")
            for lid in ALL_LINE_IDS:
                info = s.template_identities[lid]
                color = TEMPLATE_LINE_COLORS[lid]
                ground_first, ground_second = s._inferred_line_endpoints(
                    info["x_at_ref_mm"], info["theta"], y_low, y_high, s.y_ref_mm,
                )
                bev_segment = s._ground_to_bev_segment(ground_first, ground_second)
                image_segment = s._unproject_segment(bev_segment, lane_matrix)
                segment = np.asarray(
                    [[image_segment["x1"], image_segment["y1"]],
                     [image_segment["x2"], image_segment["y2"]]], dtype=np.float64
                )
                first = tuple(np.round(segment[0]).astype(int))
                if info["matched"]:
                    self._draw_segment(view, segment, color, 3)
                else:
                    self._draw_dashed_segment(view, segment, color, 1)
                cv2.putText(view, lid, (first[0] + 5, first[1] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        if s.final_pair is not None:
            p1 = s.final_pair.get("center_image_p1")
            p2 = s.final_pair.get("center_image_p2")
            if p1 is not None and p2 is not None:
                self._draw_segment(view, np.asarray([p1, p2], dtype=np.float64),
                                   (255, 0, 255), 3)
                mid = (int((p1[0] + p2[0]) * 0.5), int((p1[1] + p2[1]) * 0.5))
                cv2.putText(view, "center(new)", (mid[0] + 6, mid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 0, 255), 2, cv2.LINE_AA)

        return view

    def _draw_original_ground_ipm(self, processing_frame):
        s = self.s
        view = processing_frame.copy()
        for line in s._source_candidates:
            cv2.line(view, tuple(np.round([line["x1"], line["y1"]]).astype(int)),
                     tuple(np.round([line["x2"], line["y2"]]).astype(int)),
                     (255, 100, 0), 1, cv2.LINE_AA)
        if s.final_pair is not None:
            centre_bev = s.final_pair.get("centerline")
            if centre_bev is not None:
                lane_matrix = s._matrix_for_profile("lane")
                centre_img = s._unproject_segment(centre_bev, lane_matrix)
                centre_segment = np.asarray(
                    [[centre_img["x1"], centre_img["y1"]],
                     [centre_img["x2"], centre_img["y2"]]], dtype=np.float64
                )
                self._draw_segment(view, centre_segment, (255, 0, 255), 3)
                mid = centre_segment.mean(axis=0).astype(int)
                cv2.putText(view, "center", (mid[0] + 6, mid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 0, 255), 2, cv2.LINE_AA)
        return view

    def draw_original_view(self, processing_frame):
        if hasattr(self.s, "y_ref_mm"):
            return self._draw_original_template(processing_frame)
        return self._draw_original_ground_ipm(processing_frame)

    # ---------------- 新地面系视图 ----------------
    def render_new_ground_bev(self):
        s = self.s
        X_MIN, X_MAX = -600.0, 600.0
        Y_MIN, Y_MAX = 0.0, 3000.0
        SCALE = 0.5
        MARGIN = 40
        W = int(round((X_MAX - X_MIN) * SCALE)) + 2 * MARGIN
        H = int(round((Y_MAX - Y_MIN) * SCALE)) + 2 * MARGIN

        def g2b(x, y):
            u = MARGIN + (x - X_MIN) * SCALE
            v = H - MARGIN - (y - Y_MIN) * SCALE
            return int(round(u)), int(round(v))

        canvas = np.zeros((H, W, 3), dtype=np.uint8)
        canvas[:] = (28, 28, 28)

        for x_ref in range(-600, 601, 100):
            color = (80, 80, 80) if x_ref % 200 == 0 else (42, 42, 42)
            u1 = g2b(x_ref, Y_MIN)[0]
            u2 = g2b(x_ref, Y_MAX)[0]
            cv2.line(canvas, (u1, MARGIN), (u1, H - MARGIN), color, 1, cv2.LINE_AA)
        for y_ref in range(0, int(Y_MAX) + 1, 200):
            color = (80, 80, 80) if y_ref % 1000 == 0 else (42, 42, 42)
            v = g2b(0.0, y_ref)[1]
            cv2.line(canvas, (MARGIN, v), (W - MARGIN, v), color, 1, cv2.LINE_AA)
            cv2.putText(canvas, f"{y_ref}", (4, v + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (170, 170, 170), 1, cv2.LINE_AA)

        u0, _ = g2b(0.0, 0.0)
        yy = MARGIN
        while yy < H - MARGIN:
            ye = min(yy + 12, H - MARGIN)
            cv2.line(canvas, (u0, yy), (u0, ye), (0, 255, 255), 1, cv2.LINE_AA)
            yy += 20
        for x_ref in (-100.0, 100.0):
            uu, _ = g2b(x_ref, 0.0)
            cv2.line(canvas, (uu, MARGIN), (uu, H - MARGIN), (90, 90, 90), 1, cv2.LINE_AA)

        u0c, v0c = g2b(0.0, 0.0)
        cv2.rectangle(canvas, (u0c - 25, v0c - 12), (u0c + 25, v0c + 4),
                      (0, 190, 255), 1)

        left_pts = getattr(s, "_last_new_ground_left_pts", []) or []
        right_pts = getattr(s, "_last_new_ground_right_pts", []) or []
        center_fit = getattr(s, "_last_new_ground_center_fit", None)

        def draw_pts(pts, color, thickness=2):
            if len(pts) < 2:
                return
            for i in range(len(pts) - 1):
                p1 = g2b(pts[i][0], pts[i][1])
                p2 = g2b(pts[i + 1][0], pts[i + 1][1])
                cv2.line(canvas, p1, p2, color, thickness, cv2.LINE_AA)

        draw_pts(left_pts, (0, 255, 0))
        draw_pts(right_pts, (0, 0, 255))

        if center_fit is not None:
            a_c, b_c = center_fit
            ys = np.linspace(50.0, Y_MAX - 100.0, 20)
            cpts = [(a_c * y + b_c, y) for y in ys]
            draw_pts(cpts, (255, 0, 255), 2)
            mid = cpts[len(cpts) // 2]
            um, vm = g2b(*mid)
        return canvas

    def render_original_with_ground_centerline(self, processing_frame):
        s = self.s
        if processing_frame is None:
            return None
        view = processing_frame.copy()
        H_img, W_img = view.shape[:2]
        sx = W_img / float(s.input_width)
        sy = H_img / float(s.input_height)

        for line in s._source_candidates:
            p1 = (int(round(line["x1"] * sx)), int(round(line["y1"] * sy)))
            p2 = (int(round(line["x2"] * sx)), int(round(line["y2"] * sy)))
            cv2.line(view, p1, p2, (255, 100, 0), 1, cv2.LINE_AA)

        if s.final_pair is not None:
            for line, color in ((s.final_pair.get("left_img"), (0, 255, 0)),
                                (s.final_pair.get("right_img"), (0, 0, 255))):
                if line is None:
                    continue
                p1 = (int(round(line["x1"] * sx)), int(round(line["y1"] * sy)))
                p2 = (int(round(line["x2"] * sx)), int(round(line["y2"] * sy)))
                cv2.line(view, p1, p2, color, 3, cv2.LINE_AA)

            p1 = s.final_pair.get("center_image_p1")
            p2 = s.final_pair.get("center_image_p2")
            if p1 is not None and p2 is not None:
                q1 = (int(round(p1[0] * sx)), int(round(p1[1] * sy)))
                q2 = (int(round(p2[0] * sx)), int(round(p2[1] * sy)))
                cv2.line(view, q1, q2, (255, 0, 255), 3, cv2.LINE_AA)
                mid = (int((q1[0] + q2[0]) * 0.5), int((q1[1] + q2[1]) * 0.5))
                cv2.putText(view, "center(new-ground)", (mid[0] + 6, mid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 0, 255), 2, cv2.LINE_AA)

        cx_img = W_img // 2
        yy = 0
        while yy < H_img:
            ye = min(yy + 12, H_img)
            cv2.line(view, (cx_img, yy), (cx_img, ye), (0, 255, 255), 1, cv2.LINE_AA)
            yy += 20

        if s.final_pair is not None:
            angle_deg = float(s.final_pair.get("lane_angle_deg", 0.0))
            offset_mm = float(s.final_pair.get("offset_mm", 0.0))
            source = s.final_pair.get("theta_source", "?")
            for i, text in enumerate((
                f"yaw = {angle_deg:+.3f} deg ({source})",
                f"offset = {offset_mm:+.1f} mm",
            )):
                cv2.putText(view, text, (10, H_img - 40 + i * 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
        return view
