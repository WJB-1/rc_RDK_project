"""返场状态的业务分析器。"""

from navigation.contracts import ChoreographyAdvanceStatus, ChoreographySourceKind, ChoreographyStartStatus
from navigation.domain import AtNode
from navigation.planning import RoutePlanOutcome
from ..states import CoordinatorState
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class ReturningAnalyzer(BaseAnalyzer):
    """负责返场规划、剧本推进和返场终局。"""

    def run(self) -> AnalyzerDecision:
        """推进当前返场剧本，或在没有剧本时请求返场规划。"""
        coordinator = self._coordinator
        if coordinator.mission_finished:
            return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "返场已经完成")
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待返场动作终局")
        if coordinator.active_choreography is not None:
            active_plan = coordinator.active_choreography
            ack = coordinator._dispatch_current_action()
            if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
                if active_plan.source_kind is not ChoreographySourceKind.FINAL_RETURN:
                    navigation_state = getattr(coordinator, "_navigation_state", None)
                    robot_state = navigation_state.robot_state() if navigation_state is not None else None
                    if isinstance(getattr(robot_state, "location", None), AtNode) and robot_state.location.node_id == "J_START":
                        start_final_return = getattr(coordinator._choreographer, "start_final_return", None)
                        if start_final_return is not None:
                            final_result = start_final_return()
                            if final_result.status is ChoreographyStartStatus.STARTED:
                                coordinator._load_choreography_result(final_result)
                                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "已抵达 J_START，开始最终驶入 START")
                            coordinator.diagnostics.append("已抵达 J_START，但最终返场剧本生成失败")
                            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "最终返场剧本生成失败")
                coordinator._mission_finished = True
                return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "已完成返场剧本")
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "返场剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发返场动作")
        previous_state = coordinator.state
        if self.plan_return_route():
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "返场规划完成并装载剧本")
        if coordinator.state is not previous_state:
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "返场规划不可达")
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "返场规划暂未产生剧本")

    def plan_return_route(self) -> bool:
        """请求规划器生成返回出发区的路线，并装载编排剧本。"""
        coordinator = self._coordinator
        if coordinator._route_planner is None:
            coordinator.diagnostics.append("未装配规划器，无法生成返场路线")
            return False
        coordinator._context.active_plan = None
        coordinator._context.active_progress = None
        result = coordinator._route_planner.plan()
        if getattr(result, "outcome", None) is not RoutePlanOutcome.PLANNED or result.plan is None:
            if getattr(result, "outcome", None) is RoutePlanOutcome.TRAPPED:
                coordinator.diagnostics.append(result.reason or "返回态规划确认受困")
                coordinator._context.transition_main(CoordinatorState.ESCAPE)
            return False
        return coordinator._load_planning_result(result.plan)

    def plan(self) -> bool:
        """兼容旧的协调器重规划入口，实际业务仍由返场规划方法承载。"""
        return self.plan_return_route()

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """处理返场动作终局并继续返场分析。"""
        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的返场中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进返场流程")
        return self.run()
