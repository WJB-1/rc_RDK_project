# -*- coding: utf-8 -*-
"""
涵洞检测 + 隧道区分 (Culvert Detection)

两种检测场景:
  1. 正面涵洞: 涵洞入口在车道正前方
  2. 侧面涵洞: 涵洞位于道路侧面（路口/拐角处）

隧道判定:
  如果当前边 is_tunnel=True，检测到的洞穴结构 → 隧道，不触发涵洞
  如果 is_tunnel=False → 涵洞

当前实现: 基于 YOLO 检测框 + 地图上下文
"""


def detect_culvert(detections: list, is_tunnel: bool = False,
                   frame_shape: tuple = None) -> dict:
    """
    从 YOLO 检测结果中判定涵洞。

    :param detections: DetectionEngine.inference() 返回的检测框列表
    :param is_tunnel: 当前边是否隧道（navigation 传入的地图上下文）
    :param frame_shape: (H, W) 原图尺寸
    :return: {"detected": bool, "type": "front"|"side"|"", "confidence": float,
              "local_x_mm": float, "local_y_mm": float, "is_tunnel": bool}
    """
    # 暂时用占位逻辑：YOLO 检测到特定类别时判定为涵洞
    # 后续可根据实际涵洞标注类别调整 class_id 匹配
    _CULVERT_CLASS_IDS = {3, 4}      # 假设训练数据中 3/4 是涵洞相关类
    _TUNNEL_CLASS_IDS = {5, 6}       # 隧道墙壁相关类

    if not detections:
        return {"detected": False, "type": "", "confidence": 0.0,
                "local_x_mm": 0.0, "local_y_mm": 0.0, "is_tunnel": False}

    best = None
    best_conf = 0.0

    for d in detections:
        cls_id = d.get("class", -1)
        conf = d.get("confidence", 0.0)

        if cls_id in _CULVERT_CLASS_IDS and conf > 0.3:
            if conf > best_conf:
                best = d
                best_conf = conf
                best_is_tunnel = False
        elif cls_id in _TUNNEL_CLASS_IDS and conf > 0.3:
            if conf > best_conf:
                best = d
                best_conf = conf
                best_is_tunnel = True

    if best is None:
        return {"detected": False, "type": "", "confidence": 0.0,
                "local_x_mm": 0.0, "local_y_mm": 0.0, "is_tunnel": False}

    # 如果是隧道边且检测到隧道结构 → 不报告涵洞
    if is_tunnel and best_is_tunnel:
        return {"detected": False, "type": "", "confidence": best_conf,
                "local_x_mm": 0.0, "local_y_mm": 0.0, "is_tunnel": True}

    # 框中心位置 → 判断正面/侧面
    x1, y1, x2, y2 = best["bbox"]
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    H, W = frame_shape[:2] if frame_shape else (1080, 1920)
    culvert_type = "front" if abs(cx - W / 2) < W * 0.3 else "side"

    # 局部坐标估算 (mm) — 简化版，实际可用 IPM 或 YOLO 的距离估计
    local_x_mm = (cx - W / 2) * 2.0    # 2mm/px @ 640×640 BEV scale
    local_y_mm = cy * 2.0

    return {
        "detected": True,
        "type": culvert_type,
        "confidence": best_conf,
        "local_x_mm": local_x_mm,
        "local_y_mm": local_y_mm,
        "is_tunnel": False,
    }
