"""定义仿真世界与调试时间线使用的不可变快照。"""

from dataclasses import dataclass
from typing import Optional, Tuple, FrozenSet

from navigation.contracts import ExecutionOutcome, PerceptionFrame
from navigation.domain.state import WorldPose


@dataclass(frozen=True)
class SimMotionResult:
    """一次虚拟运动的终局及更新后的连续位姿。"""

    outcome: ExecutionOutcome
    timestamp: float
    pose: WorldPose
    odometry_delta_mm: float = 0.0
    actual_heading_deg: Optional[float] = None
    error_code: Optional[str] = None


@dataclass(frozen=True)
class SimTaskResult:
    """一次虚拟任务执行的终局。"""

    outcome: ExecutionOutcome
    timestamp: float
    task_result: Optional[str] = None
    error_code: Optional[str] = None


@dataclass(frozen=True)
class SimWorldSnapshot:
    """仿真真值世界的只读视图，不包含 RuntimeMap 状态。"""

    seed: int
    now: float
    pose: WorldPose
    truth_blocked_edge_ids: FrozenSet[str]
    truth_culvert_edge_ids: FrozenSet[str]
    observation_count: int = 0
    frame_id: Optional[str] = None

@dataclass(frozen=True)
class SimulationSnapshot:
    """SimulationRunner 对外提供的组合调试快照。"""

    running: bool
    paused: bool
    stopped: bool
    seed: int
    now: float
    world: SimWorldSnapshot
    completed_action_count: int = 0
    pending_request_id: Optional[str] = None
    last_outcome: Optional[ExecutionOutcome] = None
    last_perception_frame: Optional[PerceptionFrame] = None
    navigation: Optional[object] = None
    timeline: Tuple[object, ...] = ()
    # 新增：RobotState 的计划物理位姿，供 Web 对照 world.pose 显示
    robot_state_pose: Optional[WorldPose] = None
    # 新增：RobotState 逻辑位置的格式化标签，如 "AtNode(N1)" / "Edge(N1->T1_R, 550mm)"
    robot_location_label: Optional[str] = None
