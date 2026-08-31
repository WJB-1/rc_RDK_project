"""Navigation 2.0 的领域对象入口。

谁调用：规划器、编排器、运行时和测试。
谁响应：后续实现只从这里导入稳定的状态、任务、地图与拓扑对象。
当前内容：重新导出阶段 0 已冻结的状态、任务、动态地图和静态拓扑类型。
"""

# 重新导出位置和机器人状态类型，使调用方不依赖 `state.py` 的内部文件路径。
from .state import AtNode, LogicalLocation, OnCruiseEdge, ProgressSource, RobotState, WorldPose
# 重新导出任务和规划目标类型，使调用方不依赖 `tasks.py` 的内部文件路径。
from .tasks import Goal, GoalKind, Task, TaskKind, TaskLifecycle
# 重新导出任务注册表及其转换结果，使协调器不依赖领域内部文件路径。
from .task_registry import TaskRegistry, TaskTransition
# 重新导出运行时地图类型，使调用方不依赖 `runtime_map.py` 的内部文件路径。
from .runtime_map import AbsoluteMapUpdate, AbsoluteMapUpdateKind, MapUpdateAuthority, RuntimeMap, RuntimeMapSnapshot
# 重新导出静态拓扑类型和工厂，使调用方不依赖 `topology.py` 的内部文件路径。
from .topology import CruiseEdge, MapNode, PhysicalEdge, TrackTopology, build_default_topology

# 声明阶段 0 当前允许外部依赖的领域类型，后续类型将在对应任务完成后补充。
__all__ = (
    # 位置、位姿和机器人状态类型。
    "ProgressSource",
    "AtNode",
    "OnCruiseEdge",
    "LogicalLocation",
    "WorldPose",
    "RobotState",
    # 任务、目标及其枚举类型。
    "TaskKind",
    "TaskLifecycle",
    "GoalKind",
    "Task",
    "Goal",
    # 任务生命周期的唯一受控注册表和每次状态转换结果。
    "TaskTransition",
    "TaskRegistry",
    # 动态地图事实、来源权限、快照和唯一写入者类型。
    "MapUpdateAuthority",
    "AbsoluteMapUpdateKind",
    "AbsoluteMapUpdate",
    "RuntimeMapSnapshot",
    "RuntimeMap",
    # 静态端口拓扑、物理边、巡航视图和工厂类型。
    "MapNode",
    "PhysicalEdge",
    "CruiseEdge",
    "TrackTopology",
    "build_default_topology",
)
