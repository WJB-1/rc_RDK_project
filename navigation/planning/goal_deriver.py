"""定义从长期比赛任务派生短期规划目标的纯规则。"""

# 导入 Python 3.8 兼容的元组类型，候选目标不以可变列表跨模块传递。
from typing import Tuple

# 导入领域公开对象，派生器只读取静态拓扑和任务快照，不写运行时地图。
from navigation.domain import Goal, GoalKind, Task, TaskKind, TaskLifecycle, TrackTopology


class GoalDeriver:
    """将单个待办任务展开为一个或两个可供路线规划器比较的短期目标。

    谁调用：后续 `Coordinator` 读取 `TaskRegistry.pending_tasks()` 后调用。
    谁响应：本类根据任务类型和只读拓扑返回不可变 `Goal` 元组。
    输入输出：输入一个 `Task`；输出一个、两个或零个可选择目标。
    状态影响：不修改任务生命周期、静态拓扑、动态地图或执行计划。
    """

    def __init__(self, topology: TrackTopology) -> None:
        """保存只读拓扑依赖，供任务目标标识转换为路口候选。"""

        # 保存静态拓扑引用；其公开查询不会产生运行时状态变化。
        self._topology = topology

    def derive(self, task: Task) -> Tuple[Goal, ...]:
        """从一项任务派生当前允许参与路线选择的短期目标。

        谁调用：后续 `Coordinator` 或规划测试。
        谁响应：本类按打卡或涵洞语义构造稳定候选目标。
        输入输出：输入任务快照；输出零个、一个或两个不可变 `Goal`。
        状态影响：只读任务与拓扑，不推进任务生命周期。
        """

        # 非待办任务不能重新进入普通规划候选，防止重复打卡或侦查。
        if task.lifecycle is not TaskLifecycle.PENDING:
            return ()
        # 打卡目标直接就是一个路口中心，验证其存在后派生唯一目标。
        if task.kind is TaskKind.CHECK_IN:
            self._topology.get_node(task.target_id)
            return (
                Goal(
                    goal_id="{}@{}".format(task.task_id, task.target_id),
                    kind=GoalKind.TASK_ARRIVAL,
                    arrival_node_id=task.target_id,
                    task_id=task.task_id,
                ),
            )
        # 涵洞目标位于物理道路上，必须由道路反查得到两个相邻路口候选。
        if task.kind is TaskKind.CULVERT_RECON:
            cruise_edges = self._topology.get_cruise_edges_for_physical_edge(task.target_id)
            # 每个方向的巡航终点都是可在路口中心执行任务的一侧候选。
            goals = tuple(
                Goal(
                    goal_id="{}@{}".format(task.task_id, cruise_edge.to_junction),
                    kind=GoalKind.TASK_ARRIVAL,
                    arrival_node_id=cruise_edge.to_junction,
                    task_id=task.task_id,
                    approach_edge_id=task.target_id,
                )
                for cruise_edge in cruise_edges
            )
            # 以目标标识稳定排序，避免道路原始定义方向影响候选次序。
            return tuple(sorted(goals, key=lambda goal: goal.goal_id))
        # 枚举扩展但未同步派生规则时显式失败，避免任务被静默遗漏。
        raise ValueError("不支持的任务类型")
