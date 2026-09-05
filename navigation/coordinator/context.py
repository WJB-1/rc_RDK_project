"""保存 Coordinator 分层状态机运行期间的可变上下文。"""

from dataclasses import dataclass
from typing import Any, Optional

from .states import (
    CoordinatorState,
    DepartureSubstate,
    EscapeSubstate,
    ReturnSubstate,
    TaskSubstate,
)


@dataclass(frozen=True)
class LastMotionRecord:
    """记录最近一次成功运动的最小摘要，供跨流程恢复判断。"""

    # 成功动作的稳定标识。
    action_id: str
    # 动作命令类型名称，避免上下文依赖完整 Action 对象。
    command_type: str
    # 当前动作是否属于转弯动作。
    is_turn: bool
    # 动作关联的巡航边；非边运动时为空。
    traversal_id: Optional[str] = None
    # 前向转弯使用的标定轨迹；非转弯时为空。
    forward_trajectory_id: Optional[str] = None

    @property
    def source_action_id(self) -> str:
        """返回兼容旧接口的转弯来源动作标识。"""

        return self.action_id


@dataclass(frozen=True)
class PendingRetrace:
    """记录等待生成撤回转弯剧本的来源动作。"""

    source_action_id: str


class CoordinatorContext:
    """集中保存外层状态、内部阶段和当前异步执行生命周期。"""

    def __init__(self) -> None:
        """创建从出发编排开始的空运行上下文。"""

        self.state = CoordinatorState.DEPARTURE
        self.departure_substate = DepartureSubstate.PREPARE
        self.task_substate: Optional[TaskSubstate] = None
        self.escape_substate: Optional[EscapeSubstate] = None
        self.return_substate: Optional[ReturnSubstate] = None
        self.active_plan: Any = None
        self.active_progress: Any = None
        self.current_action: Any = None
        self.current_request: Any = None
        self.last_motion: Optional[LastMotionRecord] = None
        self.pending_retrace: Optional[PendingRetrace] = None
        self.diagnostics = []

    def transition(self, state: CoordinatorState, substate=None) -> None:
        """原子切换外层状态和对应子状态，拒绝跨状态使用错误枚举。"""

        expected = {
            CoordinatorState.DEPARTURE: DepartureSubstate,
            CoordinatorState.TASK_PROCESSING: TaskSubstate,
            CoordinatorState.ESCAPE: EscapeSubstate,
            CoordinatorState.RETURNING: ReturnSubstate,
        }.get(state)
        if expected is not None and not isinstance(substate, expected):
            raise ValueError("{} 必须使用 {} 子状态".format(state.value, expected.__name__))
        if state is CoordinatorState.EXCEPTION and substate is not None:
            raise ValueError("EXCEPTION 不应携带内部子状态")
        self.state = state
        self.departure_substate = substate if state is CoordinatorState.DEPARTURE else None
        self.task_substate = substate if state is CoordinatorState.TASK_PROCESSING else None
        self.escape_substate = substate if state is CoordinatorState.ESCAPE else None
        self.return_substate = substate if state is CoordinatorState.RETURNING else None
