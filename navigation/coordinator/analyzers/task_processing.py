"""任务处理状态的业务分析器。"""

from navigation.contracts import ChoreographyAdvanceStatus
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
        result = coordinator._route_planner.plan()
        if getattr(result, "outcome", None) is RoutePlanOutcome.PLANNED and result.plan is not None:
            return coordinator._load_planning_result(result.plan)
        return False

    def handle_map_update(self, update):
        """任务分析器不再判断路线影响；该业务统一由编排器返回明确结果。"""

        return None

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """处理任务动作终局并继续任务分析。"""
        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的任务中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进任务流程")
        return self.run()
