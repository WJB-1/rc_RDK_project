"""任务处理主状态的隐式分析流程。"""

from navigation.contracts import ChoreographyAdvanceStatus
from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class TaskProcessingAnalyzer(BaseAnalyzer):
    """负责任务规划、剧本推进和完成后的下一步判断。"""

    def run(self) -> AnalyzerDecision:
        """优先推进活动剧本，没有剧本时请求一次普通规划。"""

        coordinator = self._coordinator
        if coordinator.mission_finished:
            return AnalyzerDecision(AnalyzerDecisionKind.TERMINAL, "导航任务已经完成")
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待任务动作终局")
        if coordinator.active_choreography is not None:
            previous_state = coordinator.state
            ack = coordinator._dispatch_current_action()
            if coordinator._last_choreography_status is ChoreographyAdvanceStatus.FINISHED:
                if coordinator.state is not previous_state:
                    return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务剧本完成，进入下一主状态")
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务剧本完成，等待下一次规划")
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "任务剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发任务动作")
        previous_state = coordinator.state
        if coordinator._task_flow.plan():
            if coordinator.state is not previous_state:
                return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务规划完成并装载剧本")
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务规划完成")
        if coordinator.state is not previous_state:
            return AnalyzerDecision(AnalyzerDecisionKind.TRANSITIONED, "任务规划不可达，进入脱困")
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "普通规划暂未产生可执行剧本")

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """消费任务动作中断后继续任务分析。"""

        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的任务中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进任务流程")
        return self.run()
