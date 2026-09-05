"""脱困状态机的业务流程入口。"""

from navigation.contracts import ChoreographyStartStatus
from navigation.domain import AbsoluteMapUpdate, AbsoluteMapUpdateKind, MapUpdateAuthority
from navigation.planning import RecoveryPlanOutcome


class EscapeFlow:
    """封装方向分析、侧支试探和倒车恢复协调。"""

    def __init__(self, coordinator) -> None:
        """保存公共协调器服务。"""

        self._coordinator = coordinator

    def assess_and_replan(self, retry_normal=True):
        """请求协调器执行当前受困分析和恢复分流。"""

        if retry_normal and self._coordinator._try_normal_plan():
            return True
        planner = self._coordinator._route_planner
        if planner is None:
            self._coordinator.diagnostics.append("未装配正常规划器，无法进行受困分析")
            return False
        assess_method = getattr(planner, "assess_escape", None)
        if assess_method is None:
            self._coordinator.diagnostics.append("规划器未提供受困分析接口")
            return False
        assessment = assess_method()
        if self._coordinator._is_worth_trying(assessment.forward):
            if self._coordinator._try_normal_plan():
                return True
        for report, side in ((assessment.left, "LEFT"), (assessment.right, "RIGHT")):
            if not self._coordinator._is_worth_trying(report):
                continue
            if self.try_side(self._coordinator._turn_direction_by_name(side)):
                return True
        recovery_planner = self._coordinator._recovery_planner
        if recovery_planner is None:
            self._coordinator.diagnostics.append("没有可尝试的前向方向且未装配恢复规划器")
            return False
        recovery_result = recovery_planner.plan()
        if recovery_result.outcome is not RecoveryPlanOutcome.RECOVERABLE or recovery_result.plan is None:
            self._coordinator.diagnostics.append("恢复规划未找到可行倒车目标")
            return False
        return self._coordinator._load_planning_result(recovery_result.plan)

    def try_side(self, side):
        """请求编排器通过协调器启动指定侧支局部剧本。"""

        return self._coordinator.start_junction_escape(side)

    def dispatch_next(self):
        """提交脱困剧本的下一条异步动作。"""

        return self._coordinator.dispatch_next()

    def handle_interrupt(self, interrupt):
        """把脱困动作终局交回协调器统一校验。"""

        return self._coordinator._handle_execution_interrupt_core(interrupt)

    def start_retrace_turn(self, source_action_id):
        """装载编排器按原轨迹生成的撤回转弯剧本。"""

        coordinator = self._coordinator
        from .states import CoordinatorState, EscapeSubstate
        coordinator._context.transition(CoordinatorState.ESCAPE, EscapeSubstate.RETRACE_TURN)
        start_method = getattr(coordinator._choreographer, "start_retrace_turn", None)
        if start_method is None:
            coordinator.diagnostics.append("编排器未提供撤回转弯入口")
            return False
        result = start_method(source_action_id)
        if result.status is not ChoreographyStartStatus.STARTED or result.plan is None or result.progress is None:
            coordinator.diagnostics.append("撤回转弯剧本启动被拒绝")
            return False
        coordinator._context.active_plan = result.plan
        coordinator._context.active_progress = result.progress
        coordinator.diagnostics.append("已装载同轨迹撤回转弯剧本")
        return True

    def handle_blocked_action(self, action):
        """写入阻塞道路事实、销毁失效剧本并进入脱困状态。"""

        coordinator = self._coordinator
        traversal_id = None
        if action is not None:
            command = action.command
            traversal_id = getattr(command, "traversal_id", None)
            if traversal_id is None:
                traversal_id = getattr(command, "target_traversal_id", None)
        if traversal_id is not None and coordinator._navigation_state is not None and coordinator._topology is not None:
            try:
                from_node_id, to_node_id = traversal_id.split("->", 1)
                cruise_edge = coordinator._topology.get_cruise_edge(from_node_id, to_node_id)
                for edge_id in cruise_edge.physical_edge_ids:
                    coordinator._navigation_state.apply_map_update(
                        AbsoluteMapUpdate(AbsoluteMapUpdateKind.BLOCK_EDGE, MapUpdateAuthority.COORDINATOR, edge_id=edge_id)
                    )
            except (KeyError, ValueError) as error:
                coordinator.diagnostics.append("阻塞巡航无法映射物理边：{}".format(error))
        else:
            coordinator.diagnostics.append("阻塞动作缺少巡航边或地图拓扑，未写入物理阻塞事实")
        coordinator._context.active_plan = None
        coordinator._context.active_progress = None
        from .states import CoordinatorState, EscapeSubstate
        coordinator._context.transition(CoordinatorState.ESCAPE, EscapeSubstate.RESTORE_HEADING)
        coordinator._mark_pending_replan()
        coordinator.diagnostics.append("动作阻塞，已销毁剧本并等待重新规划")
