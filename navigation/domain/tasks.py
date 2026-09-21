"""定义比赛任务、短期规划目标及其显式生命周期。"""

# 导入不可变数据类装饰器，避免任务和目标在跨模块传递时被原地修改。
from dataclasses import dataclass
# 导入枚举基类，保证任务语义和生命周期使用受控值。
from enum import Enum
# 导入 Python 3.8 兼容的可选值类型注解。
from typing import Optional


class TaskKind(Enum):
    """比赛任务的业务类型，任务注册表据此决定完成条件和执行目标。"""

    # 表示到达指定路口并完成打卡。
    CHECK_IN = "check_in"
    # 表示从涵洞入口相邻路口执行涵洞侦查。
    CULVERT_RECON = "culvert_recon"


class TaskLifecycle(Enum):
    """任务生命周期，任务注册表只允许沿既定顺序管理状态。"""

    # 表示任务已知但尚未提交给任务执行器。
    PENDING = "pending"
    # 表示任务已提交并等待任务系统异步中断。
    EXECUTING = "executing"
    # 表示已收到匹配任务请求的成功中断。
    COMPLETED = "completed"


class GoalKind(Enum):
    """短期规划目标的来源，目标只表达终点位置而不等同于长期任务。"""

    # 表示由待完成任务派生的到达路口目标。
    TASK_ARRIVAL = "task_arrival"
    # 表示所有任务结束后明确返回 START 出发区。
    RETURN_TO_START = "return_to_start"


@dataclass(frozen=True)
class Task:
    """比赛目标的长期业务对象，任务选择器和任务注册表围绕它工作。"""

    # 稳定任务标识，供请求、中断和日志交叉引用。
    task_id: str
    # 打卡或涵洞侦查等任务类型。
    kind: TaskKind
    # 任务对应的静态节点或物理边标识。
    target_id: str
    # 任务当前生命周期，只有任务注册表可改变其业务含义。
    lifecycle: TaskLifecycle = TaskLifecycle.PENDING


@dataclass(frozen=True)
class Goal:
    """一次规划使用的短期到达目标，由任务或返航规则派生。

    谁调用：任务选择器或返航规则创建，`RoutePlanner` 消费。
    谁响应：规划器将 `arrival_node_id` 作为当前路线的最终路口中心。
    输入输出：输入为目标来源、最终路口和可选来源任务；输出为一次规划的语义终点。
    状态影响：只描述规划意图，不计算收益、不修改任务生命周期。
    """

    # 本次规划目标的稳定标识。
    goal_id: str
    # 任务到达或返回出发区等目标来源。
    kind: GoalKind
    # 规划器必须最终到达的路口中心标识，不是路径中自动经过的节点列表。
    arrival_node_id: str
    # 若目标由任务派生则记录来源任务，返回出发区时为空。
    task_id: Optional[str] = None
    # 涵洞任务在目标路口需要面对的物理道路；打卡和返航目标没有该限制。
    approach_edge_id: Optional[str] = None
    # 探索目标要求路线最后必须驶过该有向巡航边；普通任务和返回目标保持为空。
    required_final_traversal_id: Optional[str] = None
