"""返场主状态的隐式分析流程。"""

from navigation.contracts import ChoreographyAdvanceStatus
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class ReturningAnalyzer(BaseAnalyzer):
    """负责返场路线、尾段剧本和终局判断。"""

    def run(self) -> AnalyzerDecision:
        """有返场剧本时提交动作，没有时请求返场规划。"""

        coordinator = self._coordinator
        if coordinator.mission_finished:
            return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "返场已经完成")
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待返场动作终局")
        if coordinator.active_choreography is not None:
            previous_state = coordinator.state
            ack = coordinator._dispatch_current_action()
            if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
                if coordinator.mission_finished:
                    return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "已完成返场剧本")
                if coordinator.state is not previous_state:
                    return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "返场剧本完成")
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "返场剧本已结束")
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "返场剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发返场动作")
        previous_state = coordinator.state
        if coordinator._return_flow.plan():
            if coordinator.state is not previous_state:
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "返场规划完成并装载剧本")
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "返场规划完成")
        if coordinator.state is not previous_state:
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "返场规划不可达")
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "返场规划暂未产生可执行剧本")

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """消费返场动作中断后继续返场分析。"""

        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的返场中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进返场流程")
        return self.run()
