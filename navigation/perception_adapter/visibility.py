"""定义可标定的摄像头可见性参数和涵洞覆盖区间推导。"""

from dataclasses import dataclass
from typing import Tuple

from navigation.contracts import CoverageInterval


@dataclass(frozen=True)
class PerceptionVisibilityProfile:
    """用命名比例描述固定摄像头一次观察能够照明的边段。"""

    junction_start_ratio: float = 0.0
    junction_end_ratio: float = 0.35
    zone_start_ratio: float = 0.0
    zone_end_ratio: float = 0.35

    def __post_init__(self) -> None:
        """校验待标定比例，避免产生越界覆盖事实。"""
        for value in (self.junction_start_ratio, self.junction_end_ratio, self.zone_start_ratio, self.zone_end_ratio):
            if not 0.0 <= value <= 1.0:
                raise ValueError("可见性比例必须位于 [0.0, 1.0]")
        if self.junction_start_ratio > self.junction_end_ratio or self.zone_start_ratio > self.zone_end_ratio:
            raise ValueError("可见性区间起点不能晚于终点")

    def coverage(self, junction_observation: bool) -> Tuple[CoverageInterval, ...]:
        """返回一次观察对应的涵洞覆盖区间。"""
        if junction_observation:
            return (CoverageInterval(self.junction_start_ratio, self.junction_end_ratio),)
        return (CoverageInterval(self.zone_start_ratio, self.zone_end_ratio),)
