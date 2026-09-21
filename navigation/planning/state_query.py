"""定义规划器读取导航现场的最小只读端口。"""

from typing import Protocol, Tuple

from navigation.domain import RobotState, RuntimeMapSnapshot, Task

from .models import PlanningPhase


class PlanningReadPort(Protocol):
    """向规划器提供不可变现场快照，不暴露任务或地图的写入能力。"""

    def robot_state(self) -> RobotState: ...

    def runtime_map_snapshot(self) -> RuntimeMapSnapshot: ...

    def pending_tasks(self) -> Tuple[Task, ...]: ...

    def mission_phase(self) -> PlanningPhase: ...

    def mission_finished(self) -> bool: ...
