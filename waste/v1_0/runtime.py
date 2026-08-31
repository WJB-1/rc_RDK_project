"""导航运行时的总入口。

这个文件不实现视觉识别、最短路算法或具体的电机控制。它更像一个
“总调度员”：接收外部事件，把事件交给正确的模块处理，再提供当前
应该执行的动作。

一次典型的数据流是：

    外部事件 -> dispatch() -> 更新状态/推进动作
    需要新路线 -> plan() -> ControlLayer -> RoutePlanner -> Choreographer
    底盘执行动作 -> ActionFeedback -> dispatch() -> 下一个动作

阅读本文件时，重点看“谁被创建”和“事件被转发给谁”，而不是寻找
具体的路径规划公式。
"""

from dataclasses import dataclass
from typing import Callable, Iterable, Optional

from .contracts import (ActionFeedback, ActionPlan, Goal, MapUpdateIntent,
                        NavigationPose, OdomUpdate)
from .control.choreographer import Choreographer
from .control.control_layer import ControlLayer
from .domain.cost_policy import CostPolicy
from .domain.mission_state import MissionState
from .domain.runtime_map import RuntimeMap
from .domain.topology import RaceTrackTopology, get_topology
from .planning.directed_dijkstra import DirectedDijkstra
from .planning.route_planner import RoutePlanner


@dataclass(frozen=True)
class RuntimeSnapshot:
    """给界面、日志或调试工具读取的运行时只读快照。"""

    pose: NavigationPose
    map_version: int
    active_action_id: str = ""
    started: bool = False


class NavigationRuntime:
    """导航系统的总入口和模块组装点。

    外部程序只需要和这个类交互：用 ``dispatch(event)`` 报告事件，
    用 ``plan()`` 请求动作计划，用 ``next_action()`` 读取当前动作。
    本类持有其他模块，但不替它们实现内部算法。
    """

    def __init__(self, topology: RaceTrackTopology | None = None,
                 clock: Callable[[], float] | None = None):
        # 静态赛道结构；未传入时使用项目默认拓扑。
        self.topology = topology or get_topology()
        # 时钟做成可注入函数，仿真时可以使用确定性的 SimClock。
        self.clock = clock or __import__("time").time
        # 动态地图和任务状态是导航运行期间持续维护的事实数据。
        self.runtime_map = RuntimeMap()
        self.mission_state = MissionState()
        # 规划请求需要的机器人位姿与当前位置。
        self.pose = NavigationPose()
        self.current_node = "START"
        # 运行时控制状态，以及可选的真实底盘适配器。
        self._started = False
        self._actuator = None
        self._goal_supplier: Callable[[], Iterable[Goal]] = lambda: ()
        # 编排器维护动作计划和当前动作游标。
        self.choreographer = Choreographer()
        # 依赖组装：控制层选择目标，规划器计算路线，编排器生成动作。
        self.control = ControlLayer(
            RoutePlanner(DirectedDijkstra(self.topology)),
            choreographer=self.choreographer,
            runtime_map=self.runtime_map,
            mission_state=self.mission_state,
            policy=CostPolicy.explore(),
            pose=self.pose,
            start_node=self.current_node,
        )

    def set_goal_supplier(self, supplier: Callable[[], Iterable[Goal]]) -> None:
        """注册目标提供函数，控制层规划时会调用它获取候选目标。"""
        self._goal_supplier = supplier
        self.control.set_goal_supplier(supplier)

    def set_actuator(self, actuator) -> None:
        """保存底盘适配器；实际 UART/ROS 发送由外部执行器完成。"""
        self._actuator = actuator

    def start(self) -> None:
        """启动运行时，并初始化控制层。"""
        self._started = True
        self.control.start()

    def dispatch(self, event) -> Optional[ActionPlan]:
        """统一接收外部事件，并把事件路由给对应模块。

        调用方不直接修改地图、任务状态或动作队列，而是把“发生了什么”
        包装成事件传进来。
        """
        if isinstance(event, OdomUpdate):
            # 里程计只更新位姿，不单独触发重新规划。
            self.pose.x_mm += event.dx_mm
            self.pose.y_mm += event.dy_mm
            self.pose.yaw_deg += event.dyaw_deg
            self.control.pose = self.pose
            return None
        if isinstance(event, MapUpdateIntent):
            # 由运行时统一写入动态地图；新事实会使旧路线失效。
            if self.runtime_map.apply(event):
                self.control.on_plan_interrupted("map_update")
            return None
        if isinstance(event, ActionFeedback):
            # 底盘反馈交给编排器推进当前动作游标。
            self.choreographer.step(event)
            if event.status in {"failed", "cancelled", "interrupted"}:
                # 动作失败：清除旧路线，等待恢复或重新规划。
                self.control.on_plan_interrupted(event.reason or event.status)
            elif self.choreographer.current_action is None:
                # 所有动作完成：通知控制层本段路线结束。
                self.control.on_plan_finished(event)
            return None
        if getattr(event, "kind", "") in {"obstacle", "timeout", "actuator_failure"}:
            # 紧急事件先取消当前动作，再让控制层进入中断状态。
            self.choreographer.cancel(getattr(event, "kind", ""))
            self.control.on_plan_interrupted(getattr(event, "kind", ""))
            return None
        # 未知事件不会擅自改变导航状态。
        return None

    def plan(self) -> Optional[ActionPlan]:
        """请求控制层完成“选目标、算路线、编排动作”这一业务链。"""
        if not self._started:
            # 未启动时不能产生动作，防止底盘被误驱动。
            return None
        navigation_plan = self.control.tick()
        if navigation_plan is None:
            # 没有候选目标、没有可达路线或控制层尚未准备好。
            return None
        return self._current_action_plan()

    def next_action(self):
        """返回动作计划中当前游标指向的动作。"""
        return self.choreographer.current_action

    def _current_action_plan(self) -> Optional[ActionPlan]:
        """读取编排器当前计划；这是运行时内部辅助方法。"""
        active = self.choreographer.active_plan
        if active is None:
            return None
        return active

    def snapshot(self) -> RuntimeSnapshot:
        """返回一份供界面、日志和调试使用的状态快照。"""
        action = self.next_action()
        return RuntimeSnapshot(self.pose, self.runtime_map.snapshot().version,
                               action.action_id if action else "", self._started)
