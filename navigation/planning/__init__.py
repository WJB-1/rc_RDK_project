"""Navigation 2.0 的纯规划公共入口。"""

# 重新导出任务候选派生器，使协调器不依赖规划模块的内部文件路径。
from .goal_deriver import GoalDeriver
# 重新导出正常路线规划器，使协调器不依赖具体搜索算法文件路径。
from .route_planner import RoutePlanner
# 重新导出受限恢复规划器，使协调器不依赖具体倒车验证文件路径。
from .recovery_planner import RecoveryPlanner
# 重新导出路线和恢复数据包，使协调器不依赖规划模型的内部文件路径。
from .models import (
    RecoveryPlan,
    RecoveryPlanOutcome,
    RecoveryPlanResult,
    RecoveryQuery,
    RecoveryStep,
    RecoveryStepKind,
    RoutePlan,
    RoutePlanOutcome,
    RoutePlanResult,
    RouteQuery,
    RouteStep,
)

# 声明本阶段唯一允许外部依赖的规划类型，后续路线与恢复类型将按工作包补充。
__all__ = (
    # 从长期任务派生短期规划候选的纯规则对象。
    "GoalDeriver",
    # 正常路线规划的有向 Dijkstra 实现。
    "RoutePlanner",
    # 只验证进入边并产生单步倒车恢复语义的规划器。
    "RecoveryPlanner",
    # 正常路线查询、步骤、计划和显式结果类型。
    "RoutePlanOutcome",
    "RouteQuery",
    "RouteStep",
    "RoutePlan",
    "RoutePlanResult",
    # 恢复查询、步骤、计划和显式结果类型。
    "RecoveryPlanOutcome",
    "RecoveryStepKind",
    "RecoveryQuery",
    "RecoveryStep",
    "RecoveryPlan",
    "RecoveryPlanResult",
)
