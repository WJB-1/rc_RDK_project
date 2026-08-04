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
from .algorithms.crossroad_detect import detect_crossroad as _ipm_crossroad
from .algorithms.crossroad_detect import get_last_duty as _get_duty
from .algorithms.obstacle_detect import detect_obstacle as _detect_obs
from .algorithms.culvert_detect import detect_culvert as _detect_culvert


class VisionToolsImpl:
    """
    VisionTools 接口实现

    整合:
    - models/yolo_detect.py: YOLOv8 BPU 检测引擎
    - algorithms/crossroad_detect.py: IPM 占空比路口检测
    - algorithms/obstacle_detect.py: 前向障碍物检测
    - algorithms/culvert_detect.py: 涵洞检测+隧道区分
    """

    def __init__(self, yolo_model_path: str = None):
        """
        :param yolo_model_path: YOLO .bin 模型路径，None 则不加载（纯 IPM 模式）
        """
        self._yolo = None
        if yolo_model_path:
            from .models.yolo_detect import DetectionEngine
            self._yolo = DetectionEngine(model_path=yolo_model_path)

    # ================================================================
    # detect_crossroad
    # ================================================================
    def detect_crossroad(self, frame, bev_mask=None,
                         ipm_engine=None, y_top=0, y_bottom=0,
                         canvas_w=600) -> CrossroadDetection:
        """
        路口检测 — IPM 占空比法 + YOLO 融合。

        如果 YOLO 可用，同时跑 YOLO 检测作为补充。
        """
        boxes = []

        # YOLO 检测（可选）
        if self._yolo is not None:
            dets = self._yolo.inference(frame)
            boxes = [DetectionBox(
                class_id=d["class"],
                class_name=str(d["class"]),
                confidence=d["confidence"],
                x1=d["bbox"][0], y1=d["bbox"][1],
                x2=d["bbox"][2], y2=d["bbox"][3],
            ) for d in dets]

        # IPM 占空比法（主要信号）
        detected = False
        distance_mm = -1.0
        duty = 0.0

        if bev_mask is not None and ipm_engine is not None:
            detected, duty = _ipm_crossroad(bev_mask, y_top, y_bottom, canvas_w)
            if detected:
                distance_mm = (ipm_engine.new_canvas_h - y_top) / ipm_engine.pixel_per_mm

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
