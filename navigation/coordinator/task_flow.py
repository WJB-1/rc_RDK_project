"""任务处理状态机的业务流程入口。"""

from navigation.contracts import ChoreographyStartStatus
from navigation.domain import AbsoluteMapUpdateKind, TaskKind


class TaskFlow:
    """封装规划、编排、执行和分析更新的任务闭环。"""

    def __init__(self, coordinator) -> None:
        """保存公共协调器服务。"""

        self._coordinator = coordinator

    def plan(self):
        """在规划门禁满足时请求一次正常规划。"""

        from .states import CoordinatorState, EscapeSubstate, TaskSubstate
        self._coordinator._context.transition(CoordinatorState.TASK_PROCESSING, TaskSubstate.PLANNING)
        if self._coordinator._try_normal_plan():
            return True
        self._coordinator._context.transition(CoordinatorState.ESCAPE, EscapeSubstate.ASSESS)
        return self._coordinator._escape_flow.assess_and_replan(retry_normal=False)

    def dispatch_next(self):
        """提交当前任务剧本的下一条异步动作。"""

        return self._coordinator.dispatch_next()

    def handle_interrupt(self, interrupt):
        """把任务动作终局交回协调器统一投影和分流。"""

        return self._coordinator._handle_execution_interrupt_core(interrupt)

    def handle_map_update(self, update):
        """接收事件投影器写入成功后的地图事实，并处理路线生命周期。"""

        if update.kind is AbsoluteMapUpdateKind.DISCOVER_CULVERT:
            return self._replace_culvert_choreography(update)
        return self._handle_map_update_impact(update)

    def _replace_culvert_choreography(self, update):
        """将命中当前路线的涵洞发现交给编排器替换剩余剧本。"""

        coordinator = self._coordinator
        if (
            coordinator._plan is None
            or coordinator._progress is None
            or coordinator._topology is None
            or update.edge_id is None
        ):
            coordinator.diagnostics.append("涵洞发现缺少活动剧本或拓扑，未替换编排")
            return
        traversal_id = None
        for candidate in coordinator._plan.route_steps:
            try:
                from_node_id, to_node_id = candidate.split("->", 1)
                cruise_edge = coordinator._topology.get_cruise_edge(from_node_id, to_node_id)
            except (KeyError, ValueError):
                continue
            if update.edge_id in cruise_edge.physical_edge_ids:
                traversal_id = candidate
                break
        if traversal_id is None:
            return
        task_id = None
        if coordinator._task_registry is not None:
            for task in coordinator._task_registry.pending_tasks():
                if task.kind is TaskKind.CULVERT_RECON and task.target_id == update.edge_id:
                    task_id = task.task_id
                    break
        if task_id is None:
            coordinator.diagnostics.append("活动路线发现涵洞但没有匹配的待办涵洞任务：{}".format(update.edge_id))
            return
        replace_method = getattr(coordinator._choreographer, "replace_current_traversal_with_culvert", None)
        if replace_method is None:
            coordinator.diagnostics.append("编排器未提供涵洞剧本替换接口")
            return
        result = replace_method(coordinator._plan, coordinator._progress, traversal_id, task_id)
        if result.status is not ChoreographyStartStatus.STARTED or result.plan is None or result.progress is None:
            coordinator.diagnostics.append("涵洞剧本替换被拒绝")
            return
        coordinator._plan = result.plan
        coordinator._progress = result.progress
        coordinator._context.active_plan = result.plan
        coordinator._context.active_progress = result.progress
        coordinator.diagnostics.append("已将活动巡航替换为涵洞探索剧本：{}".format(task_id))

    def _handle_map_update_impact(self, update):
        """根据已生效的地图事实决定保留剧本还是等待重新规划。"""

        coordinator = self._coordinator
        if update.kind is not AbsoluteMapUpdateKind.BLOCK_EDGE:
            return
        if coordinator._plan is None or coordinator._topology is None or update.edge_id is None:
            return
        for traversal_id in coordinator._plan.route_steps:
            from_node_id, to_node_id = traversal_id.split("->", 1)
            cruise_edge = coordinator._topology.get_cruise_edge(from_node_id, to_node_id)
            if update.edge_id not in cruise_edge.physical_edge_ids:
                continue
            coordinator._mark_pending_replan()
            if coordinator._plan.source_kind.value == "junction_recovery":
                if coordinator._context.last_motion is not None and coordinator._context.last_motion.is_turn:
                    from .context import PendingRetrace
                    coordinator._context.pending_retrace = PendingRetrace(
                        coordinator._context.last_motion.source_action_id
                    )
                coordinator._plan = None
                coordinator._progress = None
                coordinator._context.active_plan = None
                coordinator._context.active_progress = None
                coordinator.diagnostics.append("侧支观察发现阻塞，等待生成同轨迹撤回剧本")
                return
            coordinator._plan = None
            coordinator._progress = None
            coordinator._context.active_plan = None
            coordinator._context.active_progress = None
            coordinator.diagnostics.append("地图更新影响活动路线，已销毁剧本并等待重新规划")
            return
