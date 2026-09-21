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
    # YOLO 标签映射:
    #   class 1 = culvert_entrance (涵洞口) → M3 处理
    #   class 2 = tunnel_entrance (隧道口) → M3 处理
    #   class 3 = wall (涵洞壁/隧道壁) → 由 is_tunnel 参数区分
    #     - is_tunnel=False → 涵洞侧墙 (side culvert)
    #     - is_tunnel=True  → 隧道侧墙 (忽略)
    _WALL_CLASS_ID = 3
    _ENTRANCE_CLASS_IDS = {1, 2}  # 涵洞口/隧道口 (M3 处理)

    if not detections:
        return {"detected": False, "type": "", "confidence": 0.0,
                "local_x_mm": 0.0, "local_y_mm": 0.0, "is_tunnel": False}

    best_wall = None
    best_wall_conf = 0.0
    best_entrance = None
    best_entrance_conf = 0.0

    for d in detections:
        cls_id = d.get("class", -1)
        conf = d.get("confidence", 0.0)

        if cls_id == _WALL_CLASS_ID and conf > 0.3:
            if conf > best_wall_conf:
                best_wall = d
                best_wall_conf = conf
        elif cls_id in _ENTRANCE_CLASS_IDS and conf > 0.3:
            if conf > best_entrance_conf:
                best_entrance = d
                best_entrance_conf = conf

    # 优先墙壁检测（标签3）
    best = best_wall
    best_conf = best_wall_conf
    is_entrance = False

    if best is None and best_entrance is not None:
        # 仅有入口检测 → 转发给 M3（M2 不处理入口）
        best = best_entrance
        best_conf = best_entrance_conf
        is_entrance = True

    if best is None:
        return {"detected": False, "type": "", "confidence": 0.0,
                "local_x_mm": 0.0, "local_y_mm": 0.0, "is_tunnel": False}

    # 如果是隧道边 → 隧道侧墙（忽略，不触发涵洞）
    if is_tunnel and not is_entrance:
        # 隧道内墙壁 = 正常，返回 detected=False 但标记 is_tunnel=True
        return {"detected": False, "type": "", "confidence": best_conf,
                "local_x_mm": 0.0, "local_y_mm": 0.0, "is_tunnel": True}

    # 框中心位置 → 判断正面/侧面
    x1, y1, x2, y2 = best["bbox"]
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    H, W = frame_shape[:2] if frame_shape else (1080, 1920)
    if is_entrance:
        culvert_type = "front"  # 入口检测永远正面
    else:
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
