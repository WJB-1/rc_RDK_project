"""
perception/source.py — L0 采集层接口（DetectionSource）

留白（D-10）：真车相机的 DetectionSource 实现尚未接。
当前仿真用 perception/simulation/ 里的 VisionChecker 作为平行实现。

设计接口见 docs/architecture/边级巡航控制层详细设计.md L0 层：
产出 RawDetection{label, 位置, 距离, 置信度, 时间戳}，不懂语义、不懂地图。
"""
from typing import Protocol, List


class RawDetection:
    """原始检测结果（值对象），由采集层产出、适配层消费。"""
    def __init__(self, label: str, confidence: float,
                 distance_mm: float = 0.0, local_x_mm: float = 0.0,
                 local_y_mm: float = 0.0, timestamp: float = 0.0):
        self.label = label            # 原始标签（如 "wall" / "culvert_front"）
        self.confidence = confidence
        self.distance_mm = distance_mm
        self.local_x_mm = local_x_mm
        self.local_y_mm = local_y_mm
        self.timestamp = timestamp


class DetectionSource(Protocol):
    """采集层接口：真车相机 / 仿真伪造 都实现此接口。"""

    def detect(self, frame) -> List[RawDetection]:
        """一帧 → 一组原始检测。留白，待真车接入。"""
        ...
