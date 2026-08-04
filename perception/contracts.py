"""
perception/contracts.py — 感知守护进程的数据结构与接口契约

perception 作为独立线程常驻运行：
- 每帧 BiSeNet 分割 + IPM → offset_mm
- 始终发送 CMD_LANE_OFFSET 给下位机（PID 车道保持）
- 中断式上报重大事件给 navigation
"""

from dataclasses import dataclass, field
from typing import Optional, Protocol, List, Tuple


# ================================================================
# 数据结构
# ================================================================

@dataclass
class LaneState:
    """
    单帧车道状态 — IPM 三段式状态机的输出

    这是 perception 的核心数据产品。
    """
    pid_error_mm: float = 0.0            # 横向偏差 (mm), 负=偏左需右修
    crossroad_detected: bool = False     # IPM 是否检测到路口
    distance_to_crossroad_mm: float = -1.0  # 到路口距离
    lane_angle_rad: float = 0.0          # 车道方向角 (rad)
    quality_score: float = 1.0           # 帧质量 0~1
    frame_dropped: bool = False          # 是否丢帧
    drop_reason: str = ""                # 丢帧原因
    duty_cycle: float = 0.0              # 路口占空比


@dataclass
class PerceptionFrame:
    """
    perception 每帧生产的完整数据包

    navigation 通过 on_road_condition() 接收此结构，
    不必每帧响应，但可用于判断当前道路状况。
    """
    lane_state: LaneState = field(default_factory=LaneState)
    offset_mm: float = 0.0
    is_intersection: bool = False
    distance_to_crossroad_mm: float = -1.0
    quality_score: float = 1.0
    timestamp: float = 0.0


# ================================================================
# 接口契约 (Protocol)
# ================================================================

class LaneTracker(Protocol):
    """车道追踪器契约"""

    def process(self, frame) -> Tuple[float, bool, 'np.ndarray']:
        """
        处理单帧 → (offset_mm, is_intersection, debug_frame)
        """
        ...

    def reset(self) -> None: ...


class PerceptionAdapter(Protocol):
    """感知适配层契约 — 连接 perception 与 navigation"""

    def on_road_condition(self, pf: PerceptionFrame) -> None:
        """每帧调用，传递道路状况"""
        ...

    def on_crossroad_detected(self, distance_mm: float,
                              duty_cycle: float) -> None:
        """中断式上报：检测到路口"""
        ...
