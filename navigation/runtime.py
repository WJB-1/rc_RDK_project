"""提供导航子系统的浅应用门面。"""

# 导入不可变数据类，保证调试快照创建后不会被调用方原地修改。
from dataclasses import dataclass
# 导入 Python 3.8 兼容的可选值和只读元组类型。
from typing import Optional, Tuple

from .coordinator import Coordinator


@dataclass(frozen=True)
class NavigationRuntimeSnapshot:
    """描述导航门面和 Coordinator 当前现场的只读快照。

    谁调用：`RobotRuntime`、`SimulationRunner` 或调试面板。
    谁响应：`NavigationRuntime.snapshot()` 汇总 Coordinator 的公开只读属性。
    输入输出：不接收参数，返回不可变快照；诊断列表转换为元组。
    状态影响：只读取现场，不创建请求、不推进剧本、不修改导航状态。
    """

    # 门面是否已经完成启动。
    started: bool
    # Coordinator 当前外层状态。
    state: object
    # Coordinator 当前外层状态对应的子阶段。
    substate: Optional[object]
    # 当前唯一在途动作。
    current_action: Optional[object]
    # 当前唯一在途执行请求。
    current_request: Optional[object]
    # 当前活动编排剧本。
    active_choreography: Optional[object]
    # 是否正在等待执行器返回终局中断。
    is_waiting_interrupt: bool
    # 最近一次成功运动的最小摘要。
    last_completed_turn: Optional[object]
    # Coordinator 诊断信息的不可变副本。
    diagnostics: Tuple[str, ...]


class NavigationRuntime:
    """导航系统的浅门面，不管理 Executor，也不转发执行中断。

    谁调用：外层 `RobotRuntime`、`SimulationRunner` 和调试入口。
    谁响应：本类只维护门面生命周期并读取 Coordinator 的公开查询属性。
    输入输出：构造时接收已装配的 Coordinator；`start()` 无返回值，`snapshot()` 返回快照。
    状态影响：启动不会自动规划或提交动作，执行器回调由 Coordinator 自己注册。
    """

    def __init__(self, coordinator: Coordinator) -> None:
        """接收一个已完成依赖装配的 Coordinator。"""

        # Runtime 必须有一个可提供导航只读查询的 Coordinator。
        if not isinstance(coordinator, Coordinator):
            raise TypeError("coordinator 必须是 Coordinator")
        # 保存唯一的业务协调器引用，不复制其现场状态。
        self._coordinator = coordinator
        # 记录门面是否已经被外层运行时启动。
        self._started = False

    @property
    def started(self) -> bool:
        """返回门面是否已经启动。"""

        return self._started

    def start(self) -> None:
        """标记导航门面已就绪，不自动发送第一条动作。

        重复调用保持幂等，不重置 Coordinator，也不重新注册执行器回调。
        """

        self._started = True

    def snapshot(self) -> NavigationRuntimeSnapshot:
        """汇总 Coordinator 的公开只读属性并返回不可变调试快照。"""

        return NavigationRuntimeSnapshot(
            started=self._started,
            state=self._coordinator.state,
            substate=self._coordinator.substate,
            current_action=self._coordinator.current_action,
            current_request=self._coordinator.current_request,
            active_choreography=self._coordinator.active_choreography,
            is_waiting_interrupt=self._coordinator.is_waiting_interrupt,
            last_completed_turn=self._coordinator.last_completed_turn,
            diagnostics=tuple(self._coordinator.diagnostics),
        )
