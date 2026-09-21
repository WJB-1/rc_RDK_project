# -*- coding: utf-8 -*-
"""
前向障碍物检测 (Obstacle Detection)

当前方案: 基于 YOLOv8 检测框 + 简单的深度估算。
YOLO 如果检测到特定类别（未在路口类中的大物体），且框面积大+位置居中，
则判定为前向障碍物。

后续可扩展为超声波/深度相机融合。
"""

import numpy as np

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


def is_obstacle_in_lane(bbox, seg_mask, min_overlap_ratio=0.3):
    """
    判断障碍物 bbox 是否在当前车道延长线上。

    逻辑: bbox 底部 1/3 区域与 seg_mask 车道线区域做交集，
          重叠比例 > min_overlap_ratio → 障碍物在车道上。

    Args:
        bbox: dict or DetectionBox with x1, y1, x2, y2 keys
        seg_mask: (H, W) uint8 — 语义分割 mask, 255=车道线区域
        min_overlap_ratio: 最小重叠比例阈值

    Returns:
        bool: True if obstacle overlaps lane area
    """
    if seg_mask is None:
        return False
    h, w = seg_mask.shape[:2]
    # Get bbox coordinates (support DetectionBox, dict with bbox list, and dict with individual keys)
    if hasattr(bbox, 'x1'):
        x1, y1, x2, y2 = bbox.x1, bbox.y1, bbox.x2, bbox.y2
    elif "bbox" in bbox:
        # YOLO output dict: {"class": ..., "confidence": ..., "bbox": [x1, y1, x2, y2]}
        x1, y1, x2, y2 = bbox["bbox"]
    else:
        x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]
    # Take bottom 1/3 of bbox (where obstacle touches ground)
    bbox_bottom_y1 = int(y1 + (y2 - y1) * 2 // 3)
    bbox_bottom_y2 = int(y2)
    # Clamp to image bounds
    x1_c = max(0, int(x1))
    x2_c = min(w, int(x2))
    y1_c = max(0, bbox_bottom_y1)
    y2_c = min(h, bbox_bottom_y2)
    if x2_c <= x1_c or y2_c <= y1_c:
        return False
    bbox_region = seg_mask[y1_c:y2_c, x1_c:x2_c]
    overlap_pixels = np.count_nonzero(bbox_region == 255)
    total_pixels = bbox_region.size
    return (overlap_pixels / total_pixels) > min_overlap_ratio
