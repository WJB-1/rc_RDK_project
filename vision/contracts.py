"""
vision/contracts.py — 视觉工具库的数据结构与接口契约

vision 模块是无状态的工具函数集合。
所有方法由 navigation 主动调用，不做决策。
"""

from dataclasses import dataclass
from typing import Optional, List, Protocol


# ================================================================
# 数据结构
# ================================================================

@dataclass
class DetectionBox:
    """
    单个检测框 — YOLO/其他检测器的通用输出格式
    """
    class_id: int                    # 类别 ID
    class_name: str = ""             # 类别名称
    confidence: float = 0.0          # 置信度
    x1: int = 0                      # 左上 X (像素, 相对原图)
    y1: int = 0                      # 左上 Y
    x2: int = 0                      # 右下 X
    y2: int = 0                      # 右下 Y

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1


@dataclass
class CrossroadDetection:
    """
    路口检测结果 — detect_crossroad() 返回
    """
    detected: bool
    confidence: float = 0.0
    distance_mm: float = -1.0         # 到路口横向线的估算距离
    duty_cycle: float = 0.0           # 占空比
    boxes: List[DetectionBox] = None   # 原始的 YOLO 检测框

    def __post_init__(self):
        if self.boxes is None:
            self.boxes = []


@dataclass
class CulvertDetection:
    """
    涵洞检测结果 — detect_culvert() 返回
    """
    detected: bool
    confidence: float = 0.0
    local_x_mm: float = 0.0           # 车体坐标系下 X (mm)
    local_y_mm: float = 0.0           # 车体坐标系下 Y (mm)
    is_wall_detection: bool = False   # True=侧面墙壁检测, False=正面入口检测
    boxes: List[DetectionBox] = None

    def __post_init__(self):
        if self.boxes is None:
            self.boxes = []


@dataclass
class ObstacleDetection:
    """
    障碍物检测结果 — detect_obstacle() 返回
    """
    detected: bool
    confidence: float = 0.0
    distance_mm: float = 0.0          # 到障碍物距离 (mm)
    box: Optional[DetectionBox] = None


@dataclass
class CulvertReconResult:
    """
    涵洞侦查结果（人脸+OCR）
    """
    face_detected: bool = False
    face_id: str = ""                 # 人脸 ID
    confidence: float = 0.0
    ocr_text: str = ""               # OCR 文本
    image_saved: bool = False         # 是否保存了截图


# ================================================================
# 接口契约 (Protocol)
# ================================================================

class VisionTools(Protocol):
    """
    vision 工具库契约 — 与 navigation/contracts.py 中的定义一致。

    所有方法无状态、无副作用。
    """

    def detect_crossroad(self, frame) -> CrossroadDetection:
        """
        YOLO 路口检测。
        :param frame: BGR numpy array (H, W, 3)
        :return: CrossroadDetection（detected=False 表示未检测到）
        """
        ...

    def detect_culvert(self, frame,
                       is_tunnel: bool = False) -> CulvertDetection:
        """
        涵洞检测。
        :param frame: BGR numpy array
        :param is_tunnel: 当前边是否为隧道（由 navigation 传入）
        :return: CulvertDetection（detected=False 表示未检测到）
        """
        ...

    def detect_obstacle(self, frame) -> ObstacleDetection:
        """
        前向障碍物检测。
        :param frame: BGR numpy array
        :return: ObstacleDetection（detected=False 表示未检测到）
        """
        ...
