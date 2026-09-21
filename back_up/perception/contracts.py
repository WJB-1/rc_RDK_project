"""
perception/contracts.py — 感知守护进程的数据结构与接口契约

perception 作为独立线程常驻运行：
- 每帧 BiSeNet 分割 + IPM → offset_mm
- 始终发送 CMD_LANE_OFFSET 给下位机（PID 车道保持）
- 中断式上报重大事件给 navigation
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional, Protocol, Tuple


class VisionHealthState(Enum):
    """视觉资源的运行健康状态。"""

    STOPPED = "stopped"
    STARTING = "starting"
    READY = "ready"
    RUNNING = "running"
    DEGRADED = "degraded"
    FAILED = "failed"


class LaneSampleStatus(Enum):
    """回正样本是否可编码为 STM32 纠偏载荷。"""

    VALID = "valid"
    PROCESSING = "processing"
    LOW_CONFIDENCE = "low_confidence"
    NO_VALID_LANE = "no_valid_lane"


@dataclass(frozen=True)
class LaneControlSample:
    """绑定 Motion 回正会话的不可变车道测量。"""

    session_id: str
    frame_id: int
    captured_at_monotonic_ns: int
    published_at_monotonic_ns: int
    status: LaneSampleStatus
    lateral_offset_mm: Optional[float]
    target_yaw_deg: Optional[float]
    quality_score: float
    reason: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.session_id:
            raise ValueError("session_id must not be empty")
        if self.frame_id < 0:
            raise ValueError("frame_id must not be negative")
        if self.captured_at_monotonic_ns < 0 or self.published_at_monotonic_ns < 0:
            raise ValueError("timestamps must not be negative")
        if self.published_at_monotonic_ns < self.captured_at_monotonic_ns:
            raise ValueError("published timestamp must not precede capture")
        if not 0.0 <= self.quality_score <= 1.0:
            raise ValueError("quality_score must be between 0 and 1")
        has_control_payload = self.lateral_offset_mm is not None and self.target_yaw_deg is not None
        if self.status is LaneSampleStatus.VALID and not has_control_payload:
            raise ValueError("VALID sample requires offset and target yaw")
        if self.status is not LaneSampleStatus.VALID and (
            self.lateral_offset_mm is not None or self.target_yaw_deg is not None
        ):
            raise ValueError("invalid sample must not carry control payload")

    @classmethod
    def valid(
        cls,
        session_id: str,
        frame_id: int,
        captured_at_monotonic_ns: int,
        published_at_monotonic_ns: int,
        quality_score: float,
        lateral_offset_mm: Optional[float] = None,
        target_yaw_deg: Optional[float] = None,
    ) -> "LaneControlSample":
        return cls(
            session_id,
            frame_id,
            captured_at_monotonic_ns,
            published_at_monotonic_ns,
            LaneSampleStatus.VALID,
            lateral_offset_mm,
            target_yaw_deg,
            quality_score,
        )

    @classmethod
    def invalid(
        cls,
        session_id: str,
        frame_id: int,
        captured_at_monotonic_ns: int,
        published_at_monotonic_ns: int,
        status: LaneSampleStatus,
        reason: str,
        quality_score: float = 0.0,
    ) -> "LaneControlSample":
        if status is LaneSampleStatus.VALID:
            raise ValueError("invalid() requires a non-VALID status")
        return cls(
            session_id,
            frame_id,
            captured_at_monotonic_ns,
            published_at_monotonic_ns,
            status,
            None,
            None,
            quality_score,
            reason,
        )


@dataclass(frozen=True)
class LaneMeasurement:
    """与 Motion 会话无关的单帧车道事实。"""

    status: LaneSampleStatus
    offset_mm: Optional[float]
    target_yaw_deg: Optional[float]
    quality_score: float
    reason: Optional[str] = None


@dataclass(frozen=True)
class FrameAnalysis:
    """视觉流水线对一帧图像产生的不可变分析结果。"""

    frame_id: int
    captured_at_monotonic_ns: int
    lane_measurement: LaneMeasurement
    is_intersection: bool = False
    distance_to_crossroad_mm: Optional[float] = None


class LaneAssistPort(Protocol):
    """Motion 使用的连续回正数据端口。"""

    def start(self, session_id: str, consumer: Callable[[LaneControlSample], None]) -> None: ...

    def stop(self, session_id: str) -> None: ...


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
