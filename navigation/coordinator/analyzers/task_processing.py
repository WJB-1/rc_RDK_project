"""任务处理主状态的隐式分析流程。"""

from .base import AnalyzerDecision, AnalyzerDecisionKind, BaseAnalyzer


class TaskProcessingAnalyzer(BaseAnalyzer):
    """负责任务规划、剧本推进和完成后的下一步判断。"""

    def run(self) -> AnalyzerDecision:
        """优先推进活动剧本，没有剧本时请求一次普通规划。"""

        coordinator = self._coordinator
        if coordinator.current_request is not None:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待任务动作终局")
        if coordinator.active_choreography is not None:
            ack = coordinator.dispatch_next()
            if ack is None:
                return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "任务剧本暂时没有可派发动作")
            return AnalyzerDecision(AnalyzerDecisionKind.DISPATCHED, "已派发任务动作")
        if coordinator._task_flow.plan():
            return self.run()
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "普通规划暂未产生可执行剧本")

    def analyze_interrupt(self, interrupt) -> AnalyzerDecision:
        """消费任务动作中断后继续任务分析。"""

        if not self._coordinator._handle_execution_interrupt_core(interrupt):
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "忽略未匹配的任务中断")
        if not self._coordinator._auto_drive:
            return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "等待外部继续推进任务流程")
        return self.run()
