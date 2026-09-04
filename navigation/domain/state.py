"""定义机器人逻辑位置、连续世界位姿和运行状态。"""

# 导入不可变数据类装饰器，保证状态更新以新值替代旧值。
from dataclasses import dataclass
# 导入枚举基类，避免用无约束字符串标识边上进度来源。
from enum import Enum
# 导入 Python 3.8 兼容的联合类型工具。
from typing import Optional, Union

from .runtime_map import AbsoluteMapUpdate, RuntimeMap, RuntimeMapSnapshot


class ProgressSource(Enum):
    """机器人边上进度的可信来源，位置投影器据此解释精度和校正方式。"""

    # 表示进度来自电控或底盘里程计的累计行驶距离。
    ODOMETRY = "odometry"
    # 表示进度已经由 IPM 或视觉地标校正。
    VISION = "vision"


@dataclass(frozen=True)
class AtNode:
    """机器人经验上位于可转向、可重规划的路口中心。"""

    # 当前路口中心或 START 出发区的静态节点标识。
    node_id: str
    # 刚刚驶入当前路口的有向巡航标识；初始位置尚无进入道路时为空。
    entry_traversal_id: Optional[str] = None


@dataclass(frozen=True)
class OnCruiseEdge:
    """机器人位于两路口中心之间的边级巡航过程，尚不处于安全重规划点。"""

    # 本次有向边级巡航的唯一标识。
    traversal_id: str
    # 本次巡航的起始路口中心标识。
    from_node_id: str
    # 本次巡航的目标路口中心标识。
    to_node_id: str
    # 已在本次巡航中累计的前进距离，单位毫米。
    progress_mm: float


# 机器人逻辑位置只能是安全路口中心，或一条巡航边上的进度位置。
LogicalLocation = Union[AtNode, OnCruiseEdge]


@dataclass(frozen=True)
class WorldPose:
    """用于仿真、面板和位置投影器的连续世界坐标位姿。"""

    # 世界坐标系中的横向位置，单位毫米。
    x_mm: float
    # 世界坐标系中的纵向位置，单位毫米。
    y_mm: float
    # 世界坐标系中的车头朝向，单位度。
    yaw_deg: float


@dataclass(frozen=True)
class RobotState:
    """导航核心持有的机器人运行状态，更新时创建新值而非原地修改。

    谁调用：`NavigationRuntime`、位置投影器和编排器。
    谁响应：规划门禁和编排器读取当前位置、朝向与待重规划标记。
    输入输出：输入为逻辑位置、朝向和进度来源；输出为可安全读取的不可变状态。
    状态影响：只有逻辑位置为 `AtNode` 时，运行时才能兑现普通重规划。
    """

    # 当前逻辑位置，决定是否可以安全转弯或兑现重规划。
    location: LogicalLocation
    # 当前车头世界朝向，供编排器选择左转、右转或直行。
    heading_deg: float
    # 当前边进度的可信数据来源。
    progress_source: ProgressSource = ProgressSource.ODOMETRY
    # 边上发现新地图事实后暂存的重规划请求，到 AtNode 才兑现。
    pending_replan: bool = False

    @property
    def is_at_safe_node(self) -> bool:
        """返回当前位置是否为可执行转弯与普通重规划的安全路口。

        谁调用：`NavigationRuntime` 的规划门禁和恢复流程。
        谁响应：本对象根据 `location` 的类型计算布尔结果。
        输入输出：不接收额外输入；返回 `True` 表示当前位置是 `AtNode`。
        状态影响：只读取状态，不修改本对象或地图。
        """

        # 只有路口中心类型代表经验上可安全转弯和重新规划的位置。
        return isinstance(self.location, AtNode)


class NavigationStateStore:
    """导航域共享状态的唯一持有者，统一维护动态地图和机器人逻辑位姿。"""

    def __init__(self, runtime_map: RuntimeMap, initial_robot_state: RobotState) -> None:
        """创建状态层并保存地图与机器人状态的两个独立内部职责。"""

        if not isinstance(runtime_map, RuntimeMap):
            raise TypeError("runtime_map 必须是 RuntimeMap")
        if not isinstance(initial_robot_state, RobotState):
            raise TypeError("initial_robot_state 必须是 RobotState")
        self._runtime_map = runtime_map
        self._robot_state = initial_robot_state

    def robot_state(self) -> RobotState:
        """返回当前机器人状态的不可变快照，调用方不能通过它修改内部状态。"""

        return self._robot_state

    def runtime_map_snapshot(self) -> RuntimeMapSnapshot:
        """返回当前动态地图的不可变快照，供规划和编排模块只读访问。"""

        return self._runtime_map.snapshot()

    def apply_map_update(self, update: AbsoluteMapUpdate) -> bool:
        """提交一条经过授权的地图事实，返回地图是否产生新变化。"""

        return self._runtime_map.apply(update)

    def replace_robot_state(self, state: RobotState) -> None:
        """由协调器提交新的完整机器人状态，状态层替换旧快照。"""

        if not isinstance(state, RobotState):
            raise TypeError("state 必须是 RobotState")
        self._robot_state = state
