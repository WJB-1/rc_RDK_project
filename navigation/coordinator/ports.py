"""定义 Coordinator 依赖的跨模块只读规划端口。"""

# 导入协议和元组类型，保证规划端口可以被真实实现和测试替身共同检查。
from typing import Protocol, Tuple

# 导入规划层的查询、结果和局部通路报告类型。
from navigation.planning import (
    EscapeAssessment,
    RecoveryPlanResult,
    RecoveryQuery,
    RoutePlanResult,
    RouteQuery,
)
from navigation.domain import Goal


class RoutePlannerPort(Protocol):
    """Coordinator 调用正常路径规划器的最小业务端口。"""

    def assess_escape(self) -> EscapeAssessment:
        """读取当前路口前后左右的局部通路事实，不选择方向。"""

    def plan(self, query: RouteQuery, candidates: Tuple[Goal, ...]) -> RoutePlanResult:
        """在候选目标中选择合法且距离最短的正常路线。"""

    def plan_to(self, query: RouteQuery, target: Goal) -> RoutePlanResult:
        """为已确定的单一目标生成正常路线。"""


class RecoveryPlannerPort(Protocol):
    """Coordinator 调用倒车恢复规划器的最小业务端口。"""

    def plan(self, query: RecoveryQuery) -> RecoveryPlanResult:
        """沿当前进入边验证并生成返回上一安全路口的恢复计划。"""

    def plan_to(self, query: RecoveryQuery, safe_node: str) -> RecoveryPlanResult:
        """为明确安全路口生成受限倒车恢复计划。"""


class ChoreographerPort(Protocol):
    """Coordinator 使用的编排器入口集合，隐藏具体阶段生成细节。"""

    def compile_next(self, plan, progress):
        """按当前游标生成唯一下一动作及其成功后游标。"""

    def start_junction_escape(self, side):
        """生成指定侧前向转弯和转弯后观察的局部剧本。"""

    def replace_current_traversal_with_culvert(self, plan, progress, traversal_id: str, task_id: str):
        """把当前巡航剩余阶段替换为涵洞探索剧本。"""
