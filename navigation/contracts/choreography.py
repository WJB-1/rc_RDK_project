"""定义编排流程、类型化动作和只读状态查询的不可变跨模块契约。"""

# 导入不可变数据类装饰器，保证协调器异步等待时流程和动作含义不会被原地修改。
from dataclasses import dataclass
# 导入受控枚举基类，避免流程阶段、动作命令和拒绝原因退化为裸字符串。
from enum import Enum
# 导入 Python 3.8 兼容的类型工具，表达可选字段、协议和联合命令类型。
from typing import Optional, Protocol, Tuple, Union

# 导入机器人不可变状态，供只读查询端口声明其唯一允许返回的动态位置数据。
from navigation.domain.state import RobotState


class ChoreographySourceKind(Enum):
    """编排剧本的来源类别，供协调器区分正常、倒车和路口撤回流程。"""

    # 表示源路线来自正常 RoutePlan。
    NORMAL = "normal"
    # 表示源路线来自 RecoveryPlan 的受限倒车步骤。
    RECOVERY = "recovery"
    # 表示源路线来自路口局部撤回决定。
    JUNCTION_RECOVERY = "junction_recovery"
    # 表示由出发分析器请求、用于 START 到 J_START 的固定启动流程。
    DEPARTURE = "departure"


class ChoreographyStageKind(Enum):
    """经验流程中的业务阶段类型，阶段本身不是可提交给执行器的机器命令。"""

    # 在进入目标巡航边前观察当前前方道路。
    OBSERVE_PRE_ENTRY = "observe_pre_entry"
    # 在路口执行一次需要由当前朝向计算左右的前向转弯。
    TURN_AT_JUNCTION = "turn_at_junction"
    # 转弯后以目标边朝向再次观察道路。
    OBSERVE_POST_TURN = "observe_post_turn"
    # 沿普通长边固定前进至观察区。
    DRIVE_TO_OBSERVATION_ZONE = "drive_to_observation_zone"
    # 在普通长边观察区执行视觉/IPM 观察。
    OBSERVE_AT_ZONE = "observe_at_zone"
    # 按当前状态中的剩余距离驶入下一路口中心。
    DRIVE_TO_NEXT_CENTER = "drive_to_next_center"
    # 执行到达任务目标后的打卡或涵洞侦查任务。
    EXECUTE_TASK = "execute_task"
    # 沿已经进入的巡航边倒车至安全路口。
    REVERSE_TO_SAFE_JUNCTION = "reverse_to_safe_junction"
    # 沿完成的前向转弯轨迹反向撤回。
    RETRACE_TURN = "retrace_turn"


class ObservationScope(Enum):
    """观察命令的业务范围，供感知系统采用对应的内部处理策略。"""

    # 在进入下一巡航边前观察当前前方。
    PRE_ENTRY = "pre_entry"
    # 在完成路口转弯后观察已经面对的目标边。
    POST_TURN = "post_turn"
    # 在普通长边的观察区执行视觉/IPM 校正。
    OBSERVATION_ZONE = "observation_zone"


class TurnDirection(Enum):
    """编排器在当前机器人朝向下计算出的合法路口转向方向。"""

    # 表示向车头左侧完成一次前向路口转弯。
    LEFT = "left"
    # 表示向车头右侧完成一次前向路口转弯。
    RIGHT = "right"


class DrivePurpose(Enum):
    """前进行驶动作的业务目的，供协调器选择对应的位置投影效果。"""

    # 表示从路口中心经验性前进至普通长边的观察区。
    TO_OBSERVATION_ZONE = "to_observation_zone"
    # 表示按当前校正后的剩余距离驶入下一路口中心。
    TO_NEXT_CENTER = "to_next_center"


class ChoreographyAdvanceStatus(Enum):
    """编排器解释一个流程指针后的显式结果类别。"""

    # 表示生成了一条当前可由协调器提交的动作。
    READY = "ready"
    # 表示当前剧本已经没有后续阶段。
    FINISHED = "finished"
    # 表示在当前状态下继续剧本不安全或配置不完整。
    REJECTED = "rejected"


class ChoreographyStartStatus(Enum):
    """编排器创建流程剧本后的显式结果类别，避免调用方猜测空对象含义。"""

    # 表示已经成功创建流程剧本和它的第一阶段指针。
    STARTED = "started"
    # 表示输入计划无法安全展开为流程剧本。
    REJECTED = "rejected"


class ChoreographyRejectionCode(Enum):
    """编排器拒绝生成动作的明确原因，协调器据此分流而不猜测。"""

    # 表示下一巡航边已经被运行时地图封锁，旧路线不能继续使用。
    STALE_BLOCKED_TRAVERSAL = "stale_blocked_traversal"
    # 表示继续路线将要求绝对禁止的 180 度掉头。
    FORBIDDEN_UTURN = "forbidden_uturn"
    # 表示执行适配器缺少已确定左/右转向的标定配置。
    MISSING_TURN_CONFIGURATION = "missing_turn_configuration"
    # 表示撤回阶段没有可以引用的已完成前向转弯动作。
    MISSING_RETRACE_SOURCE = "missing_retrace_source"
    # 表示传入的流程指针不属于当前剧本或越过有效阶段范围。
    INVALID_PROGRESS = "invalid_progress"


@dataclass(frozen=True)
class ChoreographyStage:
    """编排剧本中的一个经验业务阶段，不直接等同于一条执行命令。"""

    # 阶段在剧本中的稳定标识，供面板和日志显示。
    stage_id: str
    # 当前阶段的业务语义类型。
    kind: ChoreographyStageKind
    # 当前阶段关联的有向巡航边；任务或停止阶段可为空。
    traversal_id: Optional[str] = None
    # 当前阶段关联的路口中心；边上阶段可为空。
    node_id: Optional[str] = None
    # 任务阶段关联的任务标识；非任务阶段为空。
    task_id: Optional[str] = None
    # 撤回转弯阶段引用的已成功前向动作标识；其他阶段为空。
    source_action_id: Optional[str] = None
    # 倒车阶段成功后应抵达的安全路口；非倒车阶段为空。
    safe_node_id: Optional[str] = None


@dataclass(frozen=True)
class ChoreographyPlan:
    """由路线预展开得到的不可变经验流程剧本。

    谁调用：`Choreographer.start()` 创建，后续 `Coordinator` 保存并传回编排器。
    谁响应：`Choreographer.compile_next()` 只读取剧本和当前指针生成单条动作。
    输入输出：保存源路线、地图版本和业务阶段；输出不包含执行器请求或可变状态。
    状态影响：本对象不可变，不修改机器人、地图、任务或执行器。
    """

    # 本次编排流程的稳定标识。
    choreography_id: str
    # 源 RoutePlan、RecoveryPlan 或局部恢复决定的稳定标识。
    source_plan_id: str
    # 源路线所属的正常、倒车或局部恢复类别。
    source_kind: ChoreographySourceKind
    # 创建剧本时所依据的运行时地图版本；局部恢复可使用当前版本。
    source_map_version: int
    # 供调试显示的只读路口级步骤标识，不承担动作计算。
    route_steps: Tuple[str, ...]
    # 有序经验阶段表，不是一次性动作队列。
    stages: Tuple[ChoreographyStage, ...]


@dataclass(frozen=True)
class ChoreographyProgress:
    """指向剧本中待解释阶段的不可变流程指针。"""

    # 指针必须关联所属剧本，防止协调器把旧路线进度用于新路线。
    choreography_id: str
    # 当前将要由编排器解释的阶段下标。
    stage_index: int


@dataclass(frozen=True)
class ObserveCommand:
    """请求感知系统完成一次范围明确的单观察事务。"""

    # 当前观察位于进入边前、转弯后或普通长边观察区。
    scope: ObservationScope
    # 被本次观察确认的目标有向巡航边。
    traversal_id: str


@dataclass(frozen=True)
class TurnAtJunctionCommand:
    """请求运动系统在路口完成一次前向左转或右转。"""

    # 编排器根据实时朝向与目标边几何关系确定的合法方向。
    turn_direction: TurnDirection
    # 转弯后准备驶入的目标有向巡航边。
    target_traversal_id: str
    # 已标定的前向真实转弯轨迹；缺失时由 Coordinator 拒绝下发。
    forward_trajectory_id: Optional[str] = None
    # 与前向轨迹对应的同轨迹反向撤回轨迹；供后续局部恢复使用。
    retrace_trajectory_id: Optional[str] = None


@dataclass(frozen=True)
class DriveDistanceCommand:
    """请求运动系统沿当前巡航边前进指定距离。"""

    # 当前沿其正方向前进的有向巡航边。
    traversal_id: str
    # 本次应前进的经验距离，单位毫米。
    distance_mm: float
    # 本次前进是进入观察区还是驶入下一路口中心。
    purpose: DrivePurpose


@dataclass(frozen=True)
class ExecuteTaskCommand:
    """请求任务系统执行一项已经到达姿态要求的业务任务。"""

    # 需要由任务系统执行的稳定任务标识。
    task_id: str


@dataclass(frozen=True)
class ReverseDistanceCommand:
    """请求运动系统沿既有巡航边倒车至指定安全路口。"""

    # 当前需要沿其反方向倒退的有向巡航边。
    traversal_id: str
    # 本次计划倒退的距离，单位毫米。
    distance_mm: float
    # 倒车成功后应抵达的已知安全路口。
    safe_node_id: str


@dataclass(frozen=True)
class RetraceTurnCommand:
    """请求运动系统沿已完成前向转弯的同一实际轨迹反向撤回。"""

    # 必须引用协调器已记录的成功前向转弯动作。
    source_action_id: str


@dataclass(frozen=True)
class StopCommand:
    """请求运动系统进入停止状态，不再继续生成普通移动动作。"""

    # 停止原因，供通信适配器、日志和面板展示。
    reason: str


# 声明所有允许出现在 Action 中的类型化命令，调用者不能传入裸字符串或参数字典。
ActionCommand = Union[
    ObserveCommand,
    TurnAtJunctionCommand,
    DriveDistanceCommand,
    ExecuteTaskCommand,
    ReverseDistanceCommand,
    RetraceTurnCommand,
    StopCommand,
]


@dataclass(frozen=True)
class AwaitObservationEffect:
    """表示观察成功后先由协调器处理最终感知帧，本效果不直接改机器人状态。"""


@dataclass(frozen=True)
class AlignToTraversalEffect:
    """表示前向转弯成功后机器人已对齐准备驶入的目标巡航边。"""

    # 转弯成功后将写入路口动作上下文的目标有向巡航边。
    target_traversal_id: str


@dataclass(frozen=True)
class AdvanceOnTraversalEffect:
    """表示进入观察区成功后应按实际里程更新当前巡航边进度。"""

    # 本次前进所在的有向巡航边。
    traversal_id: str
    # 编排时的经验距离，协调器优先采用中断中的实际里程。
    planned_distance_mm: float


@dataclass(frozen=True)
class ArriveAtNodeEffect:
    """表示行驶或倒车成功后应吸附为指定路口中心。"""

    # 成功后应成为 AtNode 的路口中心标识。
    node_id: str
    # 成功进入该路口的有向巡航边；倒车时由恢复阶段决定。
    entry_traversal_id: str


@dataclass(frozen=True)
class RetraceTurnEffect:
    """表示同轨迹撤回成功后应恢复为转弯前的路口动作状态。"""

    # 被撤回的原前向转弯动作标识。
    source_action_id: str


@dataclass(frozen=True)
class CompleteTaskEffect:
    """表示任务系统成功后协调器可推进对应任务生命周期。"""

    # 成功完成的稳定任务标识。
    task_id: str


@dataclass(frozen=True)
class StopEffect:
    """表示停止动作成功后协调器不得继续生成普通后续动作。"""


# 声明所有允许出现在 Action 中的状态投影模板，模板永远不下发给执行器。
ActionExpectedEffect = Union[
    AwaitObservationEffect,
    AlignToTraversalEffect,
    AdvanceOnTraversalEffect,
    ArriveAtNodeEffect,
    RetraceTurnEffect,
    CompleteTaskEffect,
    StopEffect,
]


@dataclass(frozen=True)
class Action:
    """编排器生成的一条唯一可提交动作及其成功后状态投影模板。"""

    # 当前动作的稳定标识，协调器以它匹配最终执行中断。
    action_id: str
    # 下发给执行适配器前的类型化动作语义。
    command: ActionCommand
    # 仅由协调器在成功中断后消费的状态投影模板。
    expected_effect: ActionExpectedEffect


@dataclass(frozen=True)
class ChoreographyRejection:
    """编排器拒绝继续当前剧本时交给协调器的明确原因。"""

    # 协调器可机械分流的受控拒绝代码。
    code: ChoreographyRejectionCode
    # 面板、日志或上层错误处理可展示的具体原因。
    reason: str


@dataclass(frozen=True)
class ChoreographyStartResult:
    """包装流程剧本创建结果，供后续协调器保存剧本与首个流程指针。

    谁调用：`Choreographer.start()` 创建，后续 `Coordinator` 读取。
    谁响应：协调器按 STARTED 保存剧本与指针，按 REJECTED 上抛配置或计划错误。
    输入输出：成功时输出剧本和首个指针；拒绝时只输出明确原因。
    状态影响：本对象不可变，不提交动作、不写入机器人、地图或任务状态。
    """

    # 流程剧本是否已创建的明确结果类别。
    status: ChoreographyStartStatus
    # STARTED 时创建出的不可变经验流程剧本，其他状态必须为空。
    plan: Optional[ChoreographyPlan] = None
    # STARTED 时指向剧本第一个待解释阶段的不可变指针，其他状态必须为空。
    progress: Optional[ChoreographyProgress] = None
    # REJECTED 时携带的明确拒绝原因，其他状态必须为空。
    rejection: Optional[ChoreographyRejection] = None

    def __post_init__(self) -> None:
        """校验启动结果与可选载荷的唯一合法组合，避免协调器猜测空字段。"""

        # 成功创建时必须同时返回剧本和首个指针，且不能混入拒绝原因。
        if self.status == ChoreographyStartStatus.STARTED:
            if self.plan is None or self.progress is None or self.rejection is not None:
                raise ValueError("STARTED 结果必须同时包含剧本和首个指针，且不能包含拒绝原因")
            return
        # 创建失败时只能返回拒绝原因，不得泄漏半成品剧本或指针。
        if self.status == ChoreographyStartStatus.REJECTED:
            if self.plan is not None or self.progress is not None or self.rejection is None:
                raise ValueError("REJECTED 结果只能包含拒绝原因")
            return
        # 枚举之外的状态没有安全语义，必须显式失败。
        raise ValueError("未知的编排启动结果状态")


@dataclass(frozen=True)
class ChoreographyAdvanceResult:
    """编排器解释一次流程指针后的严格结果。

    谁调用：`Choreographer.compile_next()` 创建，后续 `Coordinator` 读取并保存动作与进度。
    谁响应：协调器按 READY、FINISHED 或 REJECTED 分别提交、结束或销毁活动流程。
    输入输出：输出至多一条动作、一个下一指针或一个拒绝原因，三者组合受状态约束。
    状态影响：本对象不可变，不自行提交执行器或修改导航状态。
    """

    # 当前编排推进的明确结果类别。
    status: ChoreographyAdvanceStatus
    # READY 时唯一可提交的动作，其他状态必须为空。
    action: Optional[Action] = None
    # READY 时动作成功后才可使用的下一指针，其他状态必须为空。
    next_progress: Optional[ChoreographyProgress] = None
    # REJECTED 时的明确原因，其他状态必须为空。
    rejection: Optional[ChoreographyRejection] = None

    def __post_init__(self) -> None:
        """校验状态与可选载荷的唯一合法组合，防止协调器猜测空字段含义。"""

        # READY 必须同时拥有动作和下一流程指针，且不能混入拒绝原因。
        if self.status == ChoreographyAdvanceStatus.READY:
            if self.action is None or self.next_progress is None or self.rejection is not None:
                raise ValueError("READY 结果必须同时包含动作和下一指针，且不能包含拒绝原因")
            return
        # FINISHED 表示剧本结束，不能留下可执行动作、旧指针或拒绝原因。
        if self.status == ChoreographyAdvanceStatus.FINISHED:
            if self.action is not None or self.next_progress is not None or self.rejection is not None:
                raise ValueError("FINISHED 结果不能包含动作、下一指针或拒绝原因")
            return
        # REJECTED 只能携带拒绝原因，不能生成任何可继续执行的载荷。
        if self.status == ChoreographyAdvanceStatus.REJECTED:
            if self.action is not None or self.next_progress is not None or self.rejection is None:
                raise ValueError("REJECTED 结果只能包含拒绝原因")
            return
        # 枚举之外的状态没有安全语义，必须显式失败。
        raise ValueError("未知的编排推进结果状态")


class NavigationStateQuery(Protocol):
    """编排器访问导航域动态状态的最小只读端口。

    谁调用：`Choreographer` 在解释流程阶段时读取。
    谁响应：后续 `NavigationRuntime` 或测试替身实现本端口。
    输入输出：返回不可变机器人状态或指定巡航边的阻塞布尔值。
    状态影响：查询不得修改机器人、地图、任务、流程或执行器。
    """

    def robot_state(self) -> RobotState:
        """返回当前不可变机器人逻辑状态。"""

    def is_traversal_blocked(self, traversal_id: str) -> bool:
        """返回指定有向巡航边是否已被运行时地图封锁。"""
