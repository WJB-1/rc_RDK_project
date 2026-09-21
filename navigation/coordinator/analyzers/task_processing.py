"""任务处理状态的业务分析器。"""

from navigation.contracts import ChoreographyAdvanceStatus
from navigation.domain import AtNode, TaskKind
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
                if coordinator.task_requirements_met:
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
        return AnalyzerDecision(AnalyzerDecisionKind.WAITING, "任务规划暂未产生剧本")

    def plan(self) -> bool:
        """执行一次正常任务路线规划。"""
        coordinator = self._coordinator
        if not coordinator._auto_drive:
            from ..states import TaskSubstate
            coordinator._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.PLANNING)
        return self.plan_normal_route()

    def plan_normal_route(self) -> bool:
        """在安全路口规划下一段任务路线；已完成多个任务时循环处理。"""

        coordinator = self._coordinator
        for _ in range(64):
            if coordinator._navigation_state is not None:
                robot_state = coordinator._navigation_state.robot_state()
                if not robot_state.is_at_safe_node:
                    coordinator.diagnostics.append(
                        "plan_normal_route 拒绝：当前位置不是安全路口，location={}".format(
                            getattr(robot_state.location, "node_id", robot_state.location)
                        )
                    )
                    return False
            if coordinator._route_planner is None:
                coordinator.diagnostics.append("plan_normal_route 拒绝：未装配正常规划器")
                return False

            result = coordinator._route_planner.plan()
            outcome = getattr(result, "outcome", None)

            if outcome is RoutePlanOutcome.PLANNED and result.plan is not None:
                steps = getattr(result.plan, "steps", None)
                if steps is not None and len(steps) == 0:
                    coordinator.diagnostics.append("plan_normal_route 拒绝：规划器返回空步骤计划")
                    return False
                coordinator.diagnostics.append(
                    "plan_normal_route 成功：steps={}".format(len(steps or ()))
                )
                return coordinator._load_planning_result(result.plan)

            if outcome is RoutePlanOutcome.TASKS_COMPLETED:
                coordinator.diagnostics.append(
                    "plan_normal_route：规划器报告任务已完成"
                )
                # 1) 当前节点匹配某个待办任务：完成它
                if self._complete_task_at_current_node(coordinator):
                    if coordinator.task_requirements_met:
                        coordinator._context.transition_main(CoordinatorState.RETURNING)
                        return False
                    continue
                # 2) 没有当前节点任务，但配额已满：进返场
                if coordinator.task_requirements_met:
                    coordinator.diagnostics.append(
                        "plan_normal_route：所有任务已完成，进入返场"
                    )
                    coordinator._context.transition_main(CoordinatorState.RETURNING)
                    return False
                # 3) 否则无可匹配任务，停止循环
                coordinator.diagnostics.append(
                    "plan_normal_route：无可匹配的待办任务，停止循环"
                )
                return False
            if outcome is RoutePlanOutcome.TRAPPED:
                coordinator.diagnostics.append(
                    "plan_normal_route 受困：{}".format(result.reason or "无原因")
                )
                coordinator._context.transition_main(CoordinatorState.ESCAPE)
                return False

            coordinator.diagnostics.append(
                "plan_normal_route 未预期结果：outcome={} plan={} reason={}".format(
                    getattr(outcome, "value", outcome),
                    result.plan is not None,
                    getattr(result, "reason", None),
                )
            )
            return False

        coordinator.diagnostics.append("plan_normal_route 达到循环上限，放弃本轮规划")
        return False

    def _complete_task_at_current_node(self, coordinator) -> bool:
        """完成与当前节点匹配的待办任务；找不到匹配时返回 False。"""

        if coordinator._task_registry is None or coordinator._navigation_state is None:
            return False
        state = coordinator._navigation_state.robot_state()
        if not isinstance(state.location, AtNode):
            return False
        node_id = state.location.node_id

        for task in coordinator._task_registry.pending_tasks():
            if task.kind is TaskKind.CHECK_IN:
                if task.target_id == node_id:
                    return self._mark_task_completed(coordinator, task.task_id)
            elif task.kind is TaskKind.CULVERT_RECON:
                topology = getattr(coordinator, "_topology", None)
                if topology is None:
                    continue
                try:
                    cruise_edges = topology.get_cruise_edges_for_physical_edge(task.target_id)
                except (KeyError, ValueError):
                    continue
                for edge in cruise_edges:
                    if edge.to_junction == node_id:
                        return self._mark_task_completed(coordinator, task.task_id)
        return False

    @staticmethod
    def _mark_task_completed(coordinator, task_id: str) -> bool:
        """把任务从 PENDING 经 EXECUTING 推到 COMPLETED。"""

        begin = coordinator._task_registry.begin(task_id)
        if not getattr(begin, "accepted", False):
            coordinator.diagnostics.append(
                "任务无法进入执行态：{}".format(getattr(begin, "reason", None))
            )
            return False
        done = coordinator._task_registry.complete(task_id)
        if getattr(done, "accepted", False):
            coordinator.diagnostics.append("任务完成：{}".format(task_id))
            return True
        coordinator.diagnostics.append(
            "任务完成状态转换被拒绝：{}".format(getattr(done, "reason", None))
        )
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