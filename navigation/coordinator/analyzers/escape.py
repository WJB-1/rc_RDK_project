"""脱困主状态的隐式分析流程。"""

from navigation.contracts import ChoreographyAdvanceStatus
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class EscapeAnalyzer(BaseAnalyzer):
    """负责恢复剧本推进，并把复杂选择委托给既有 EscapeFlow。"""

    def run(self) -> AnalyzerDecision:
        """有恢复剧本时提交动作，否则请求脱困分析和规划。"""

        coordinator = self._coordinator
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待脱困动作终局")
        if coordinator.active_choreography is not None:
            previous_state = coordinator.state
            ack = coordinator._dispatch_current_action()
            if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
                if coordinator.state is not previous_state:
                    return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困剧本完成，回到任务处理")
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "脱困剧本已结束")
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "脱困剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发脱困动作")
        previous_state = coordinator.state
        if coordinator._escape_flow.assess_and_replan():
            if coordinator.state is not previous_state:
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困决策完成并装载剧本")
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困决策完成")
        if coordinator.state is not previous_state:
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "脱困不可继续")
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "脱困分析暂未产生可执行剧本")

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """消费脱困动作中断后继续脱困分析。"""

        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的脱困中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进脱困流程")
        return self.run()
