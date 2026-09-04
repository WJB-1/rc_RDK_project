"""保存 Coordinator 分层状态机运行期间的可变上下文。"""

from typing import Any, Optional

from .states import (
    CoordinatorState,
    DepartureSubstate,
    EscapeSubstate,
    ReturnSubstate,
    TaskSubstate,
)


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

