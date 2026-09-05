"""任务处理状态机的业务流程入口。"""


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

        if update.kind.value == "discover_culvert":
            return self._coordinator._replace_culvert_choreography(update)
        return self._coordinator._handle_map_update_impact(update)
