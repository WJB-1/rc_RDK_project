"""定义 Coordinator 外层状态及各状态内部的阶段枚举。"""

from enum import Enum


class CoordinatorState(Enum):
    """协调器当前所处的业务阶段。"""

    DEPARTURE = "departure"
    TASK_PROCESSING = "task_processing"
    ESCAPE = "escape"
    RETURNING = "returning"
    EXCEPTION = "exception"


class DepartureSubstate(Enum):
    """出发状态机内部阶段。"""

    PREPARE = "prepare"
    EXECUTE = "execute"
    OBSERVE = "observe"
    RETRACE_TURN = "retrace_turn"
    SWITCH_SIDE = "switch_side"


class TaskSubstate(Enum):
    """任务处理状态机内部阶段。"""

    PLANNING = "planning"
    CHOREOGRAPHING = "choreographing"
    EXECUTING = "executing"
    ANALYZING = "analyzing"


class EscapeSubstate(Enum):
    """脱困状态机内部阶段。"""

    RESTORE_HEADING = "restore_heading"
    ASSESS = "assess"
    SELECT_DIRECTION = "select_direction"
    TURN_SIDE = "turn_side"
    OBSERVE_SIDE = "observe_side"
    RETRACE_TURN = "retrace_turn"
    BACKTRACK_PLANNING = "backtrack_planning"
    BACKTRACK_CHOREOGRAPHING = "backtrack_choreographing"
    BACKTRACK_EXECUTING = "backtrack_executing"


class ReturnSubstate(Enum):
    """返回状态机内部阶段。"""

    PLAN_TO_START = "plan_to_start"
    CHOREOGRAPH_TO_START = "choreograph_to_start"
    EXECUTE_TO_START = "execute_to_start"
    PLAN_FINAL_APPROACH = "plan_final_approach"
    CHOREOGRAPH_FINAL_APPROACH = "choreograph_final_approach"
    EXECUTE_FINAL_APPROACH = "execute_final_approach"

