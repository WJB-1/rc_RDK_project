"""定义导航层与异步执行器之间的不可变数据契约。"""

# 导入不可变数据类装饰器，保证异步等待期间请求和反馈不能被原地修改。
from dataclasses import dataclass
# 导入枚举基类，避免用无约束字符串表达环境、接收方和结果。
from enum import Enum
# 导入 Python 3.8 兼容的类型注解工具和仅类型检查开关。
from typing import TYPE_CHECKING, Optional, Tuple

# 仅供静态类型检查导入感知帧，避免执行契约与感知契约在运行时形成循环导入。
if TYPE_CHECKING:
    # 感知系统观察完成时才会使用该不可变帧类型。
    from .perception import PerceptionFrame


class ExecutionEnvironment(Enum):
    """执行器运行环境，决定导航请求交给仿真后端还是真实后端。

    谁设置：`SimulationRunner` 或真实 RDK 应用入口在启动装配时设置。
    谁读取：`IAsyncExecutor` 的路由实现。
    输入输出：作为执行器装配输入，不直接参与单条动作的数据传输。
    状态影响：运行中不得切换，避免一个动作跨越仿真与真实设备。
    """

    # 表示导航在仿真中运行，请求将交给 `SimExecutor` 和虚拟后端。
    SIMULATION = "simulation"
    # 表示导航在 RDK 真实测试中运行，请求将经由通信或真实任务业务接口。
    REAL_TEST = "real_test"


class ExecutionTarget(Enum):
    """执行请求的接收方，执行器据此路由同一格式的导航数据包。"""

    # 底盘或运动控制器，负责前进、倒车、转向和停车等实际运动。
    MOTION_CONTROLLER = "motion_controller"
    # 任务系统，负责 RFID 打卡、涵洞探索等非运动业务。
    TASK_SYSTEM = "task_system"
    # 感知系统，负责在观察动作完成后返回一帧 `PerceptionFrame`。
    PERCEPTION_SYSTEM = "perception_system"


class ExecutionOutcome(Enum):
    """异步请求的最终结果，`NavigationRuntime` 据此推进、恢复或停止计划。"""

    # 表示请求按预期完成，是动作队列可继续前进的唯一结果。
    COMPLETED = "completed"
    # 表示请求未完成且不是道路阻塞，协调器交给失败或重试策略处理。
    FAILED = "failed"
    # 表示当前运动道路不可通行，协调器将触发地图更新和倒车恢复流程。
    BLOCKED = "blocked"
    # 表示外部系统未在允许时间内完成，不得将任务或动作标记为成功。
    TIMEOUT = "timeout"
    # 表示底盘或执行设备自身异常，用于与业务失败区分。
    ACTUATOR_FAILURE = "actuator_failure"
    # 表示当前请求被正式取消，旧动作不得再推进计划。
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class ExecutionParameter:
    """动作的单个命名参数，避免用无约束字典在模块间传递含义。"""

    # 参数名，例如 `distance_mm` 或 `turn_direction`。
    name: str
    # 参数值，由 `kind` 和接收执行器共同解释。
    value: object


@dataclass(frozen=True)
class ExecutionRequest:
    """导航协调器向执行器提交的一条唯一异步动作请求。

    谁调用：`Coordinator` 或执行循环。
    谁响应：`SimExecutor` 或 `RealExecutor`。
    输入输出：输入为请求身份、接收方和动作参数；输出是对应的 `DispatchAck` 与 `ExecutionInterrupt`。
    状态影响：受理不代表完成，运行时必须等待同一身份的最终中断。
    """

    # 请求唯一标识，用于匹配受理反馈和最终中断。
    request_id: str
    # 所属执行计划标识，用于日志、取消和过期事件过滤。
    execution_id: str
    # 计划中具体动作标识，保证只推进当前这一条动作。
    action_id: str
    # 接收该请求的外部系统。
    target: ExecutionTarget
    # 动作语义名称，例如 `DRIVE_DISTANCE` 或 `OBSERVE`。
    kind: str
    # 与动作语义对应的命名不可变参数。
    parameters: Tuple[ExecutionParameter, ...]
    # 请求创建时的运行环境时间。
    timestamp: float


@dataclass(frozen=True)
class DispatchAck:
    """执行器立即返回的受理结果，不表示动作已经结束。"""

    # 被受理或拒绝的原始请求标识。
    request_id: str
    # `True` 表示后端开始处理；`False` 表示协调器不应等待完成中断。
    accepted: bool
    # 拒绝或取消受理时的简短原因。
    reason: str = ""


@dataclass(frozen=True)
class ExecutionInterrupt:
    """外部系统异步上报的一条动作最终结果。

    谁调用：下位机、任务系统或感知系统通过执行器回调。
    谁响应：`NavigationRuntime` 校验身份后交给 `Coordinator`。
    输入输出：输入为执行身份、来源、结果和可选诊断；运行时据此更新等待状态。
    状态影响：只有匹配当前动作且结果为 `COMPLETED` 的中断可以推进动作队列。
    """

    # 必须匹配当前等待请求的请求标识。
    request_id: str
    # 必须匹配当前执行计划的标识。
    execution_id: str
    # 必须匹配当前动作的标识。
    action_id: str
    # 实际产生中断的外部系统。
    source: ExecutionTarget
    # 动作完成、失败、阻塞、超时等最终结果。
    outcome: ExecutionOutcome
    # 中断产生时的运行环境时间。
    timestamp: float
    # 运动实际完成的距离，非运动请求为空。
    actual_progress_mm: Optional[float] = None
    # 转向后的实际朝向，非转向请求为空。
    actual_heading_deg: Optional[float] = None
    # 任务系统给出的简短业务结果，非任务请求为空。
    task_result: Optional[str] = None
    # 感知系统观察成功时携带的一份最终帧；其他动作与失败观察均为空。
    perception_frame: Optional["PerceptionFrame"] = None
    # 可供恢复策略和日志判断的机器错误码。
    error_code: Optional[str] = None
    # 额外的命名诊断信息，不承载未定义的业务状态。
    details: Tuple[ExecutionParameter, ...] = ()
