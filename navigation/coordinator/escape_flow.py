"""脱困状态机的业务流程入口。"""

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
