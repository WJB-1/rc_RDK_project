"""出发主状态的隐式分析流程。"""

from navigation.contracts import ChoreographyAdvanceStatus
from ..states import CoordinatorState
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class DepartureAnalyzer(BaseAnalyzer):
    """负责出发剧本的编排、执行反馈分析和任务态转移。"""

    def run(self) -> AnalyzerDecision:
        """有出发剧本时提交下一动作，否则等待外部注入剧本。"""

        coordinator = self._coordinator
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待出发动作终局")
        if coordinator.active_choreography is None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待出发剧本")
        ack = coordinator._dispatch_current_action()
        if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
            coordinator._context.transition_main(CoordinatorState.TASK_PROCESSING)
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "出发剧本完成，进入任务处理")
        if ack is None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "出发剧本暂时没有可派发动作")
        return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发出发动作")

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """消费出发动作中断后继续出发分析，不直接构造动作。"""

        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的出发中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进出发流程")
        return self.run()
