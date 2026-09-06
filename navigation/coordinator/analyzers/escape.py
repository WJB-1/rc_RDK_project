"""脱困状态的业务分析器。"""

from navigation.contracts import ChoreographyAdvanceStatus, ChoreographyStartStatus
from navigation.planning import RecoveryPlanOutcome
from ..states import CoordinatorState, EscapeSubstate
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class EscapeAnalyzer(BaseAnalyzer):
    """负责受困评估、侧支尝试、撤回转弯和倒车恢复。"""

    def run(self) -> AnalyzerDecision:
        """推进脱困剧本，或执行一次脱困方向评估。"""
        coordinator = self._coordinator
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待脱困动作终局")
        if coordinator.active_choreography is not None:
            ack = coordinator._dispatch_current_action()
            if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
                from ..states import CoordinatorState
                coordinator._context.transition_main(CoordinatorState.TASK_PROCESSING)
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困剧本完成，回到任务处理")
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "脱困剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发脱困动作")
        previous_state = coordinator.state
        if self.assess_and_replan():
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困决策完成并装载剧本")
        if coordinator.state is not previous_state:
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困不可继续")
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "脱困评估暂未产生剧本")

    def assess_and_replan(self, retry_normal=True):
        """按前、左、右优先评估方向，最后请求倒车恢复路线。"""
        coordinator = self._coordinator
        task_analyzer = coordinator._analyzers.get(CoordinatorState.TASK_PROCESSING)
        if retry_normal and task_analyzer is not None and hasattr(task_analyzer, "plan_normal_route"):
            if task_analyzer.plan_normal_route():
                return True
        planner = coordinator._route_planner
        assess_method = getattr(planner, "assess_escape", None) if planner is not None else None
        if assess_method is None:
            coordinator.diagnostics.append("规划器未提供受困分析接口")
            return False
        assessment = assess_method()
        if self._is_worth_trying(assessment.forward) and task_analyzer is not None and task_analyzer.plan_normal_route():
            return True
        for report, side in ((assessment.left, "LEFT"), (assessment.right, "RIGHT")):
            if self._is_worth_trying(report) and self.start_side_escape(self._turn_direction_by_name(side)):
                return True
        recovery_planner = coordinator._recovery_planner
        if recovery_planner is None:
            return False
        recovery_result = recovery_planner.plan()
        if recovery_result.outcome is not RecoveryPlanOutcome.RECOVERABLE or recovery_result.plan is None:
            return False
        return coordinator._load_planning_result(recovery_result.plan)

    def start_side_escape(self, side):
        """请求编排器生成指定侧支的局部脱困剧本。"""
        coordinator = self._coordinator
        if coordinator._context.current_request is not None:
            return False
        if coordinator._auto_drive:
            coordinator._context.transition_main(CoordinatorState.ESCAPE)
        else:
            coordinator._context.transition(CoordinatorState.ESCAPE, EscapeSubstate.TURN_SIDE)
        start_method = getattr(coordinator._choreographer, "start_junction_escape", None)
        if start_method is None:
            return False
        result = start_method(side)
        if result.status is not ChoreographyStartStatus.STARTED or result.plan is None or result.progress is None:
            return False
        coordinator._context.active_plan = result.plan
        coordinator._context.active_progress = result.progress
        return True

    def start_retrace_turn(self, source_action_id):
        """请求编排器沿原转弯轨迹生成撤回剧本。"""
        coordinator = self._coordinator
        start_method = getattr(coordinator._choreographer, "start_retrace_turn", None)
        if start_method is None:
            return False
        result = start_method(source_action_id)
        if result.status is not ChoreographyStartStatus.STARTED or result.plan is None or result.progress is None:
            return False
        coordinator._context.active_plan = result.plan
        coordinator._context.active_progress = result.progress
        return True

    @staticmethod
    def _is_worth_trying(report):
        """判断规划器报告的方向是否值得尝试。"""
        if hasattr(report, "worth_trying"):
            return bool(report.worth_trying)
        return getattr(report, "name", "") in ("CLEAR", "OPEN", "UNOBSERVED")

    @staticmethod
    def _turn_direction_by_name(name):
        """把方向名称转换为转向枚举。"""
        from navigation.contracts import TurnDirection
        return TurnDirection.LEFT if name == "LEFT" else TurnDirection.RIGHT

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """处理脱困动作终局并继续脱困分析。"""
        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的脱困中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进脱困流程")
        return self.run()
