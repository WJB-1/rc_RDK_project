"""Navigation 2.0 的顶层公共入口。

谁调用：应用入口、测试和后续导航模块。
谁响应：本包在后续任务中只重新导出稳定的公共领域类型。
当前内容：重新导出已冻结的领域、跨模块契约与纯编排公共入口。
"""

# 重新导出异步执行和感知数据包，使应用入口不依赖 contracts 内部文件路径。
from .contracts import ExecutionInterrupt, ExecutionRequest, PerceptionFrame
# 重新导出纯编排入口，使后续协调器不依赖 choreography 内部文件路径。
from .choreography import Choreographer, MotionProfile
# 重新导出状态、任务、地图和静态拓扑，使后续模块只依赖一个稳定入口。
from .domain import (
    AtNode,
    Goal,
    OnCruiseEdge,
    RobotState,
    RuntimeMap,
    Task,
    TrackTopology,
    build_default_topology,
)

# 声明顶层唯一允许后续模块依赖的已冻结公共类型。
__all__ = (
    # 静态拓扑与动态地图类型。
    "TrackTopology",
    "build_default_topology",
    "RuntimeMap",
    # 机器人逻辑位置、运行状态、长期任务和短期目标类型。
    "RobotState",
    "AtNode",
    "OnCruiseEdge",
    "Task",
    "Goal",
    # 异步执行和感知公共数据包。
    "ExecutionRequest",
    "ExecutionInterrupt",
    "PerceptionFrame",
    # 纯编排器与其不可变经验距离配置。
    "Choreographer",
    "MotionProfile",
)
