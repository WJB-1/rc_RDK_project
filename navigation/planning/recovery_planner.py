"""实现只允许沿已进入巡航边倒车的恢复规划，不搜索替代道路。"""

# 导入可选值和元组类型，使解析后的进入边上下文保持 Python 3.8 兼容。
from typing import Optional, Tuple

# 导入静态巡航拓扑，恢复规划器只验证进入边而不修改赛道或动态地图。
from navigation.domain import TrackTopology
# 导入恢复查询、结果和语义步骤，保证本模块不泄漏底盘动作指令。
from .models import RecoveryPlan, RecoveryPlanOutcome, RecoveryPlanResult, RecoveryQuery, RecoveryStep, RecoveryStepKind


class RecoveryPlanner:
    """从安全路口验证唯一倒车出口并生成单步 BACKTRACK 恢复计划。

    谁调用：`Coordinator` 已在安全路口调用普通 `RoutePlanner` 且得到 `NO_ROUTE` 后调用。
    谁响应：本类读取 `RecoveryQuery.location.entry_traversal_id` 和静态拓扑，返回恢复结果。
    输入输出：输入安全路口上下文；输出沿进入边回到其起点的计划或 `NO_RECOVERY`。
    状态影响：只读查询；不选择恢复场景、不写地图、不搜索替代路线且不下发动作。
    """

    def __init__(self, topology: TrackTopology) -> None:
        """保存恢复验证所需的只读静态拓扑。

        谁调用：导航装配代码在创建规划组件时调用。
        谁响应：本类保存拓扑引用供 `plan` 和 `plan_to` 查询。
        输入输出：输入 `TrackTopology`；不返回值。
        状态影响：只保存不可变赛道查询入口，不修改拓扑或机器人状态。
        """

        # 保存静态拓扑，使每次恢复都能核验进入巡航边不是伪造标识。
        self._topology = topology

    def plan(self, query: RecoveryQuery) -> RecoveryPlanResult:
        """为当前安全路口生成唯一允许的倒车恢复计划。

        谁调用：`Coordinator` 在普通规划明确无路可走时调用。
        谁响应：本类从进入巡航边解析上一安全路口并构造单步 BACKTRACK。
        输入输出：输入恢复查询；输出可恢复计划或不可恢复结果。
        状态影响：不修改查询、机器人状态、地图或任务。
        """

        # 先解析并验证进入边，缺失或伪造上下文时绝不猜测可倒车路线。
        resolved_entry = self._resolve_entry(query)
        # 解析失败时直接返回带明确原因的不可恢复结果。
        if resolved_entry is None:
            return RecoveryPlanResult(RecoveryPlanOutcome.NO_RECOVERY, None, "当前安全路口没有可验证的进入巡航边")
        # 进入边的起点就是唯一允许退回的上一安全路口。
        source_node_id, _, _ = resolved_entry
        # 复用指定安全路口接口，保证两个公开入口执行同一套验证规则。
        return self.plan_to(query, source_node_id)

    def plan_to(self, query: RecoveryQuery, safe_node: str) -> RecoveryPlanResult:
        """只在指定节点等于进入边起点时生成倒车恢复计划。

        谁调用：`Coordinator` 已明确要求退回进入边起点时调用。
        谁响应：本类验证指定节点与静态进入边，再返回单步 BACKTRACK 或 `NO_RECOVERY`。
        输入输出：输入恢复查询和目标安全节点；输出显式恢复结果。
        状态影响：不修改查询、机器人状态、拓扑或动态地图。
        """

        # 解析并验证进入边，防止错误状态把机器人倒车到不相邻或未知节点。
        resolved_entry = self._resolve_entry(query)
        # 进入边不存在、格式错误或终点不匹配时安全拒绝。
        if resolved_entry is None:
            return RecoveryPlanResult(RecoveryPlanOutcome.NO_RECOVERY, None, "当前安全路口没有可验证的进入巡航边")
        # 解包已验证的进入边起点、终点和稳定巡航标识。
        source_node_id, current_node_id, traversal_id = resolved_entry
        # 任何非进入边起点都意味着试图把恢复规划扩展成替代路线，必须拒绝。
        if safe_node != source_node_id:
            return RecoveryPlanResult(RecoveryPlanOutcome.NO_RECOVERY, None, "恢复只能退回进入巡航边的起点安全路口")
        # 用当前节点和原进入边生成稳定计划标识，便于日志、仿真和执行计划引用。
        plan_id = "recovery:{}:{}".format(current_node_id, traversal_id)
        # 创建唯一允许的回退语义步骤；traversal_id 保留原方向供后续编排器执行倒车版本。
        step = RecoveryStep(RecoveryStepKind.BACKTRACK, current_node_id, source_node_id, traversal_id)
        # 创建不包含普通路径步骤或底盘命令的受限恢复计划。
        plan = RecoveryPlan(plan_id, current_node_id, source_node_id, (step,))
        # 返回明确可恢复结果，使协调器随后可交给编排器翻译为倒车动作。
        return RecoveryPlanResult(RecoveryPlanOutcome.RECOVERABLE, plan)

    def _resolve_entry(self, query: RecoveryQuery) -> Optional[Tuple[str, str, str]]:
        """验证当前路口的进入巡航边并解析其起点、终点和标识。

        谁调用：本类的两个公开规划方法调用。
        谁响应：本方法返回经过静态拓扑验证的进入边信息，或返回空值表示不可恢复。
        输入输出：输入恢复查询；输出 `(起点, 当前终点, 巡航标识)` 或 `None`。
        状态影响：只读解析，不修改任何领域对象。
        """

        # 读取安全路口保存的历史进入边；初始位置没有该信息时无法安全倒车。
        traversal_id = query.location.entry_traversal_id
        # 缺少进入边时拒绝，不能把普通掉头误当作恢复方案。
        if traversal_id is None:
            return None
        # 有向巡航标识必须恰好由一个起终点分隔符构成，避免接受格式不完整的外部数据。
        parts = traversal_id.split("->")
        # 空端点、多余分隔符均不可能表示有效巡航边。
        if len(parts) != 2 or not parts[0] or not parts[1]:
            return None
        # 解包进入巡航边的静态起点和静态终点。
        source_node_id, destination_node_id = parts
        # 只有进入边终点就是当前位置时，才能沿该边反向退回其起点。
        if destination_node_id != query.location.node_id:
            return None
        try:
            # 从静态拓扑取得对应巡航边，确认两个路口确实相邻且可以构成该巡航。
            cruise_edge = self._topology.get_cruise_edge(source_node_id, destination_node_id)
        except KeyError:
            # 未知或不相邻节点不允许被恢复规划器临时解释为可倒车道路。
            return None
        # 拓扑派生出的稳定标识必须与状态记录完全相同，防止错误拼接方向或节点。
        if cruise_edge.traversal_id != traversal_id:
            return None
        # 返回已经完成静态验证的进入边上下文。
        return source_node_id, destination_node_id, traversal_id
