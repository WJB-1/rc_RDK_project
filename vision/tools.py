# -*- coding: utf-8 -*-
"""
vision/tools.py — VisionTools 实现

实现 contracts.VisionTools 接口。
整合 vision/models/ 和 vision/algorithms/ 下的全部视觉能力。

所有方法无状态、无副作用。navigation 调用时传入 frame + 上下文，
vision 返回结构化结果。vision 不知道状态机的存在。
"""

from pathlib import Path

from .contracts import (
    CrossroadDetection, CulvertDetection, ObstacleDetection,
    DetectionBox,
)
# IPM 方案 (保留，可选)
from .algorithms.crossroad_detect import detect_crossroad as _ipm_crossroad
from .algorithms.crossroad_detect import get_last_duty as _get_duty
# 纯分割方案 (默认)
from .algorithms.crossroad_seg import (
    detect_crossroad_from_seg as _seg_crossroad,
    confirm_crossroad_seg as _confirm_crossroad,
)
from .algorithms.obstacle_detect import detect_obstacle as _detect_obs
from .algorithms.culvert_detect import detect_culvert as _detect_culvert


class VisionToolsImpl:
    """
    VisionTools 接口实现

    整合:
    - models/yolo_detect.py: YOLOv8 BPU 检测引擎
    - algorithms/crossroad_seg.py: 纯语义分割路口检测 (默认)
    - algorithms/crossroad_detect.py: IPM 占空比路口检测 (可选)
    - algorithms/obstacle_detect.py: 前向障碍物检测
    - algorithms/culvert_detect.py: 涵洞检测+隧道区分

    路口检测策略:
      mode="seg"  → 纯分割 mask 水平占空比 + 小孔成像测距 (默认, 不依赖 IPM)
      mode="ipm"  → IPM 鸟瞰图占空比法 (需外部传入 bev_mask + ipm_engine)
    """

    def __init__(self, yolo_model_path: str = None,
                 crossroad_mode: str = "seg",
                 camera_height_mm: float = 150.0,
                 seg_roi_ratio: tuple = (0.6, 0.85)):
        """
        :param yolo_model_path: YOLO .bin 模型路径，None 则不加载
        :param crossroad_mode: "seg" (默认) 或 "ipm"
        :param camera_height_mm: 相机离地高度 (mm)，用于小孔成像测距
        :param seg_roi_ratio: 分割 mask 路口检测 ROI (y_start_ratio, y_end_ratio)
        """
        self._yolo = None
        if yolo_model_path:
            from .models.yolo_detect import DetectionEngine
            self._yolo = DetectionEngine(model_path=yolo_model_path)

        self._crossroad_mode = crossroad_mode
        self._camera_height_mm = camera_height_mm
        self._seg_roi_ratio = seg_roi_ratio

    # ================================================================
    # detect_crossroad
    # ================================================================
    def detect_crossroad(self, frame, seg_mask=None,
                         bev_mask=None, ipm_engine=None,
                         y_top=0, y_bottom=0, canvas_w=600) -> CrossroadDetection:
        """
        路口检测 — 默认使用纯分割方案，IPM 方案可选。

        Args:
            frame: BGR 图像 (H,W,3)，给 YOLO 用
            seg_mask: 语义分割二值 mask (H,W)，纯分割方案的主输入
            bev_mask: IPM 鸟瞰图 (IPM 方案用)
            ipm_engine: IPM 引擎实例 (IPM 方案用)
            y_top, y_bottom, canvas_w: IPM 方案参数

        Returns:
            CrossroadDetection
        """
        boxes = []

        # YOLO 检测（可选补充）
        if self._yolo is not None and frame is not None:
            dets = self._yolo.inference(frame)
            boxes = [DetectionBox(
                class_id=d["class"],
                class_name=str(d["class"]),
                confidence=d["confidence"],
                x1=d["bbox"][0], y1=d["bbox"][1],
                x2=d["bbox"][2], y2=d["bbox"][3],
            ) for d in dets]

        detected = False
        distance_mm = -1.0
        duty = 0.0

        if self._crossroad_mode == "seg":
            # --- 纯分割方案 (默认) ---
            if seg_mask is not None:
                h, w = seg_mask.shape[:2]
                roi_y1 = int(h * self._seg_roi_ratio[0])
                roi_y2 = int(h * self._seg_roi_ratio[1])
                frame_detected, distance_mm, duty = _seg_crossroad(
                    seg_mask, roi_y1, roi_y2,
                    camera_height_mm=self._camera_height_mm,
                )
                detected, _ = _confirm_crossroad(frame_detected)
            else:
                # 无分割 mask 时退化为 YOLO-only
                detected = False

        elif self._crossroad_mode == "ipm":
            # --- IPM 方案 (保留) ---
            if bev_mask is not None and ipm_engine is not None:
                frame_detected, duty = _ipm_crossroad(bev_mask, y_top, y_bottom, canvas_w)
                if frame_detected:
                    distance_mm = ((ipm_engine.new_canvas_h - y_top)
                                   / ipm_engine.pixel_per_mm)
                detected, _ = _confirm_crossroad(frame_detected)

        return CrossroadDetection(
            detected=detected,
            confidence=0.8 if detected else 0.0,
            distance_mm=distance_mm,
            duty_cycle=duty,
            boxes=boxes,
        )

    # ================================================================
    # detect_culvert
    # ================================================================
    def detect_culvert(self, frame, is_tunnel: bool = False) -> CulvertDetection:
        """
        涵洞检测 — YOLO + 地图上下文。
        """
        if self._yolo is None or frame is None:
            return CulvertDetection(detected=False)

        dets = self._yolo.inference(frame)
        result = _detect_culvert(dets, is_tunnel=is_tunnel,
                                 frame_shape=frame.shape[:2])

        boxes = [DetectionBox(
            class_id=d["class"],
            confidence=d["confidence"],
            x1=d["bbox"][0], y1=d["bbox"][1],
            x2=d["bbox"][2], y2=d["bbox"][3],
        ) for d in dets]

        return CulvertDetection(
            detected=result["detected"],
            confidence=result["confidence"],
            local_x_mm=result["local_x_mm"],
            local_y_mm=result["local_y_mm"],
            is_wall_detection=(result.get("type") == "side"),
            boxes=boxes,
        )

    # ================================================================
    # detect_obstacle
    # ================================================================
    def detect_obstacle(self, frame) -> ObstacleDetection:
        """
        前向障碍物检测 — YOLO + 简单几何判定。
        """
        if self._yolo is None or frame is None:
            return ObstacleDetection(detected=False)

        dets = self._yolo.inference(frame)
        result = _detect_obs(dets, frame_shape=frame.shape[:2])

        return ObstacleDetection(
            detected=result["detected"],
            confidence=result["confidence"],
            distance_mm=result["distance_mm"],
        )
