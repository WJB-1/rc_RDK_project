# -*- coding: utf-8 -*-
"""
前向障碍物检测 (Obstacle Detection)

当前方案: 基于 YOLOv8 检测框 + 简单的深度估算。
YOLO 如果检测到特定类别（未在路口类中的大物体），且框面积大+位置居中，
则判定为前向障碍物。

后续可扩展为超声波/深度相机融合。
"""

# 障碍物候选类别（非路口/涵洞/隧道类，且占画面大）= 可能是障碍物
_OBS_CLASS_THRESHOLD = 0.3          # 置信度阈值
_OBS_AREA_RATIO_MIN = 0.05          # 框面积占画面比 > 5%
_OBS_CENTER_BIAS_MAX = 0.3          # 框中心偏离画面中心 < 30%


def detect_obstacle(detections: list, frame_shape: tuple = None) -> dict:
    """
    从 YOLO 检测结果中判定正前方障碍物。

    :param detections: DetectionEngine.inference() 返回的检测框列表
    :param frame_shape: (H, W) 原图尺寸
    :return: {"detected": bool, "confidence": float, "distance_mm": float}
    """
    if not detections or frame_shape is None:
        return {"detected": False, "confidence": 0.0, "distance_mm": 0.0}

    H, W = frame_shape[:2]
    frame_area = H * W

    best_conf = 0.0
    best_dist = 0.0
    found = False

    for d in detections:
        conf = d.get("confidence", 0.0)
        if conf < _OBS_AREA_RATIO_MIN:
            continue

        x1, y1, x2, y2 = d["bbox"]
        box_area = (x2 - x1) * (y2 - y1)
        area_ratio = box_area / frame_area

        if area_ratio < _OBS_AREA_RATIO_MIN:
            continue

        # 框中心偏离判定
        cx = (x1 + x2) / 2 / W
        cy = (y1 + y2) / 2 / H
        center_bias = abs(cx - 0.5) + abs(cy - 0.3)  # 期望障碍物在中上区域

        if center_bias > _OBS_CENTER_BIAS_MAX:
            continue

        # 估算距离：框越大越近
        estimated_dist = 1000 * (1.0 - area_ratio)

        if conf > best_conf:
            best_conf = conf
            best_dist = estimated_dist
            found = True

    return {
        "detected": found,
        "confidence": best_conf,
        "distance_mm": best_dist,
    }
