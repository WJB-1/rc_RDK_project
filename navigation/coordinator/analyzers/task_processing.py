"""任务处理状态的业务分析器。"""

from navigation.contracts import ChoreographyAdvanceStatus, ChoreographyStartStatus
from navigation.domain import AbsoluteMapUpdateKind, TaskKind
from navigation.planning import RoutePlanOutcome
from ..states import CoordinatorState
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class TaskProcessingAnalyzer(BaseAnalyzer):
    """负责任务规划、剧本推进、地图更新和涵洞剧本替换。"""

    def run(self) -> AnalyzerDecision:
        """推进任务剧本；剧本耗尽后规划下一条路线。"""
        coordinator = self._coordinator
        if coordinator.mission_finished:
            return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "导航任务已经完成")
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待任务动作终局")
        if coordinator.active_choreography is not None:
            ack = coordinator._dispatch_current_action()
            if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
                if coordinator._task_registry is not None and not coordinator._task_registry.pending_tasks():
                    coordinator._context.transition_main(CoordinatorState.RETURNING)
                    return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务完成，进入返场")
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务剧本完成，等待下一次规划")
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "任务剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发任务动作")
        previous_state = coordinator.state
        if self.plan():
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务规划完成")
        if coordinator.state is not previous_state:
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务规划不可达，进入脱困")
        coordinator._context.transition_main(CoordinatorState.ESCAPE)
        return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务规划不可达，进入脱困")

    def plan(self) -> bool:
        """执行一次正常任务路线规划。"""
        coordinator = self._coordinator
        if not coordinator._auto_drive:
            from ..states import TaskSubstate
            coordinator._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.PLANNING)
        return self.plan_normal_route()

    def plan_normal_route(self) -> bool:
        """在安全路口调用规划器并装载路线编排剧本。"""
        coordinator = self._coordinator
        if coordinator._navigation_state is not None and not coordinator._navigation_state.robot_state().is_at_safe_node:
            coordinator.diagnostics.append("当前位置不是安全路口，暂不允许重新规划")
            return False
        if coordinator._route_planner is None:
            coordinator.diagnostics.append("未装配正常规划器")
            return False
        coordinator._context.active_plan = None
        coordinator._context.active_progress = None
        result = coordinator._route_planner.plan()
        if getattr(result, "outcome", None) is RoutePlanOutcome.PLANNED and result.plan is not None:
            return coordinator._load_planning_result(result.plan)
        return False

    def handle_map_update(self, update):
        """处理地图事实对任务路线的影响。"""
        if update.kind is AbsoluteMapUpdateKind.DISCOVER_CULVERT:
            return self._replace_culvert_choreography(update)
        return self._handle_map_update_impact(update)

    def _replace_culvert_choreography(self, update):
        """把当前路线中的巡航边替换为涵洞探索剧本。"""
        coordinator = self._coordinator
        if (coordinator._context.active_plan is None or coordinator._context.active_progress is None
                or coordinator._topology is None or update.edge_id is None):
            return
        traversal_id = None
        for candidate in coordinator._context.active_plan.route_steps:
            try:
                from_node_id, to_node_id = candidate.split("->", 1)
                cruise_edge = coordinator._topology.get_cruise_edge(from_node_id, to_node_id)
            except (KeyError, ValueError):
                continue
            if update.edge_id in cruise_edge.physical_edge_ids:
                traversal_id = candidate
                break
        if traversal_id is None or coordinator._task_registry is None:
            return
        task_id = next((task.task_id for task in coordinator._task_registry.pending_tasks()
                        if task.kind is TaskKind.CULVERT_RECON and task.target_id == update.edge_id), None)
        if task_id is None:
            return
        replace_method = getattr(coordinator._choreographer, "replace_current_traversal_with_culvert", None)
        if replace_method is None:
            coordinator.diagnostics.append("编排器未提供涵洞剧本替换接口")
            return
        result = replace_method(coordinator._context.active_plan, coordinator._context.active_progress, traversal_id, task_id)
        if result.status is not ChoreographyStartStatus.STARTED or result.plan is None or result.progress is None:
            coordinator.diagnostics.append("涵洞剧本替换被拒绝")
            return
        coordinator._context.active_plan = result.plan
        coordinator._context.active_progress = result.progress

    def _handle_map_update_impact(self, update):
        """阻塞当前路线时销毁剧本并标记等待重规划。"""
        coordinator = self._coordinator
        if update.kind is not AbsoluteMapUpdateKind.BLOCK_EDGE:
            return
        if coordinator._context.active_plan is None or coordinator._topology is None or update.edge_id is None:
            return
        for traversal_id in coordinator._context.active_plan.route_steps:
            from_node_id, to_node_id = traversal_id.split("->", 1)
            cruise_edge = coordinator._topology.get_cruise_edge(from_node_id, to_node_id)
            if update.edge_id not in cruise_edge.physical_edge_ids:
                continue
            coordinator._mark_pending_replan()
            coordinator._context.active_plan = None
            coordinator._context.active_progress = None
            coordinator.diagnostics.append("地图更新影响活动路线，已销毁剧本并等待重规划")
            return

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """处理任务动作终局并继续任务分析。"""
        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的任务中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进任务流程")
        return self.run()
