"""定义纯规划层的查询、正常路线、恢复路线和显式结果数据包。"""

# 导入不可变数据类装饰器，规划输入与结果在跨模块传递时不得被原地修改。
from dataclasses import dataclass
# 导入受控枚举基类，避免用无约束字符串判断规划和恢复结果。
from enum import Enum
# 导入 Python 3.8 兼容的可选值和元组类型注解。
from typing import Optional, Protocol, Tuple

# 导入安全路口、领域快照和目标，规划层只读取这些对象而不拥有其状态。
from navigation.domain import AtNode, Goal, RobotState, RuntimeMapSnapshot


class JunctionPassability(Enum):
    """当前路口某个相对方向的局部通路事实。"""

    # 静态巡航边存在，且其物理组成边都已由可靠观察确认无障碍。
    CLEAR = "clear"
    # 兼容旧调用方；OPEN 与 CLEAR 表示同一事实。
    OPEN = "clear"
    # 静态巡航边存在，但至少一条物理组成边当前被阻塞。
    BLOCKED = "blocked"
    # 静态巡航边存在，但尚未获得完整的安全或阻塞观察结论。
    UNOBSERVED = "unobserved"
    # 静态拓扑中没有对应方向的巡航边。
    ABSENT = "absent"


@dataclass(frozen=True)
class EscapeDirectionAssessment:
    """一个相对方向的通路状态和是否值得尝试的规划报告。"""

    # 该方向对应静态巡航边的动态事实状态。
    status: JunctionPassability
    # 从该方向继续后，当前任务目标或返航目标是否仍可达。
    worth_trying: bool
    # 该方向对应的有向巡航标识；不存在时为空。
    traversal_id: Optional[str] = None

    def __eq__(self, other) -> bool:
        """兼容旧的状态比较，同时保留新报告对象的完整相等语义。"""

        if isinstance(other, JunctionPassability):
            return self.status is other
        if not isinstance(other, EscapeDirectionAssessment):
            return NotImplemented
        return (
            self.status is other.status
            and self.worth_trying == other.worth_trying
            and self.traversal_id == other.traversal_id
        )


@dataclass(frozen=True)
class EscapeAssessment:
    """路径层对当前路口前后左右的事实和脱困可行性只读报告。"""

    # 机器人当前车头正前方的局部通路状态。
    forward: EscapeDirectionAssessment
    # 机器人左侧九十度方向的局部通路状态。
    left: EscapeDirectionAssessment
    # 机器人右侧九十度方向的局部通路状态。
    right: EscapeDirectionAssessment
    # 机器人后方一百八十度方向的局部通路状态，仅供诊断显示。
    backward: EscapeDirectionAssessment


@dataclass(frozen=True)
class PlanningEntryConstraint:
    """限制正常路线第一条巡航必须符合当前节点和车头朝向的规划门禁。"""

    # 规划结果必须使用的首条有向巡航标识。
    required_first_traversal_id: str
    # 该约束适用的当前路口节点。
    applicable_node_id: str
    # 构造约束时的车头朝向，单位为度。
    required_heading_deg: float


class PlanningStateQuery(Protocol):
    """规划器访问导航域共享状态的最小只读端口。"""

    def robot_state(self) -> RobotState:
        """返回当前不可变机器人状态。"""

    def runtime_map_snapshot(self) -> RuntimeMapSnapshot:
        """返回当前不可变动态地图快照。"""


class RoutePlanOutcome(Enum):
    """正常路线规划的显式结果类别，协调器据此决定继续编排或报告不可达。"""

    # 已找到一条满足阻塞与掉头约束的正常路口级路线。
    PLANNED = "planned"
    # 没有任何合法候选路线，不能用空步骤伪装成成功计划。
    NO_ROUTE = "no_route"
    # 当前规划无法满足协调器指定的首边节点或朝向约束。
    CONSTRAINT_UNSATISFIED = "constraint_unsatisfied"


class RecoveryPlanOutcome(Enum):
    """恢复规划的显式结果类别，协调器据此决定倒车恢复或进入异常处理。"""

    # 已找到能够退回安全路口的受限恢复计划。
    RECOVERABLE = "recoverable"
    # 当前路口没有经过验证的进入边或不存在允许的倒车恢复方式。
    NO_RECOVERY = "no_recovery"


class RecoveryStepKind(Enum):
    """恢复计划的语义步骤类型，后续编排器而非规划器决定具体底盘动作。"""

    # 沿当前巡航边反向倒退至进入该边前的安全路口。
    BACKTRACK = "backtrack"


@dataclass(frozen=True)
class RouteQuery:
    """一次正常路径搜索所需的安全路口、方向上下文和动态地图快照。

    谁调用：后续 `Coordinator` 在规划门禁满足且机器人位于 `AtNode` 时构造。
    谁响应：`RoutePlanner` 读取本对象进行有向 Dijkstra 搜索。
    输入输出：输入起点、进入巡航和车头朝向；地图由规划器装配的只读状态口读取。
    状态影响：本对象不修改机器人、地图、任务或拓扑。
    """

    # 当前安全路口中心或 START 的静态标识。
    start_node_id: str
    # 刚驶入当前路口的有向巡航；初始位置尚无进入道路时为空。
    entry_traversal_id: Optional[str]
    # 当前车头朝向，初始位置没有进入巡航时用于检查首段掉头风险。
    heading_deg: float
    # 可选的首边规划门禁，普通自由规划时为空。
    entry_constraint: Optional[PlanningEntryConstraint] = None


@dataclass(frozen=True)
class RouteStep:
    """正常路线中的一个相邻路口巡航步骤，不表达机器转向或行驶距离。"""

    # 本次巡航的起始路口中心。
    from_junction: str
    # 本次巡航的目标路口中心。
    to_junction: str
    # 对应 `TrackTopology.CruiseEdge` 的稳定有向标识。
    traversal_id: str


@dataclass(frozen=True)
class RoutePlan:
    """一条可交给后续编排器解释的正常路口级路线。

    谁调用：`RoutePlanner` 创建，后续 `Coordinator` 和 `Choreographer` 读取。
    谁响应：后续编排器将其路口步骤与机器人朝向翻译为执行计划。
    输入输出：保存规划依据、选中目标和有序步骤；不保存机器动作。
    状态影响：路线是不可变建议，不修改地图、任务或机器人位置。
    """

    # 本次正常路线的稳定标识，供日志和后续执行计划交叉引用。
    plan_id: str
    # 生成路线时所依据的动态地图版本。
    map_version: int
    # 路线起点，必须等于相应 `RouteQuery.start_node_id`。
    start_node_id: str
    # 本次从候选中选中的短期目标。
    selected_goal: Goal
    # 依次通过的相邻路口巡航步骤。
    steps: Tuple[RouteStep, ...]
    # 所有巡航边物理长度之和，单位毫米，不含未实现的转向成本。
    total_distance_mm: float


@dataclass(frozen=True)
class RoutePlanResult:
    """包装正常路线成功或不可达结果，避免调用方猜测空步骤的业务含义。"""

    # 成功或不可达的明确结果类别。
    outcome: RoutePlanOutcome
    # 仅成功时携带完整路线；不可达时为空。
    plan: Optional[RoutePlan]
    # 不可达时供日志和面板显示的简短原因；成功时为空。
    reason: Optional[str] = None


@dataclass(frozen=True)
class RecoveryQuery:
    """一次受限倒车恢复所需的当前安全路口与进入边上下文。

    谁调用：后续 `Coordinator` 在 `AtNode` 的普通规划返回 `NO_ROUTE` 后构造。
    谁响应：`RecoveryPlanner` 读取本对象决定是否能沿进入边倒退。
    输入输出：输入当前安全路口及其进入巡航标识；输出恢复结果。
    状态影响：本对象不取消执行计划，也不写入动态地图。
    """

    # 当前安全路口；其 entry_traversal_id 是恢复唯一允许沿用的历史巡航边。
    location: AtNode


@dataclass(frozen=True)
class RecoveryStep:
    """恢复路线的单个语义步骤，后续编排器负责翻译为倒车动作。"""

    # 当前仅支持沿原巡航边退回的 BACKTRACK 语义。
    kind: RecoveryStepKind
    # 恢复步骤开始时所在的当前安全路口。
    from_junction: str
    # 恢复完成后必须到达的安全路口。
    to_junction: str
    # 引用原进入巡航标识，明确本步骤是该边的倒车版本而不是普通新路线。
    traversal_id: str


@dataclass(frozen=True)
class RecoveryPlan:
    """一条只允许退回安全路口的恢复语义计划，不是普通路线或底盘命令。"""

    # 本次恢复计划的稳定标识，供日志和后续执行计划交叉引用。
    plan_id: str
    # 触发恢复时所在的当前安全路口；普通规划已在这里返回 NO_ROUTE。
    start_node_id: str
    # 恢复后必须重新吸附到的安全路口。
    safe_node_id: str
    # 当前仅包含 BACKTRACK 的恢复语义步骤。
    steps: Tuple[RecoveryStep, ...]


@dataclass(frozen=True)
class RecoveryPlanResult:
    """包装恢复成功或不可恢复结果，避免用普通路线表示倒车失败。"""

    # 可恢复或不可恢复的明确结果类别。
    outcome: RecoveryPlanOutcome
    # 仅可恢复时携带恢复计划；不可恢复时为空。
    plan: Optional[RecoveryPlan]
    # 不可恢复时供日志和面板显示的简短原因；成功时为空。
    reason: Optional[str] = None
