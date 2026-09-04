"""任务处理状态机的业务流程入口。"""


class TaskFlow:
    """封装规划、编排、执行和分析更新的任务闭环。"""

    def __init__(self, coordinator) -> None:
        """保存公共协调器服务。"""

        self._coordinator = coordinator

    def plan(self):
        """在规划门禁满足时请求一次正常规划。"""

        return self._coordinator.replan()

    def dispatch_next(self):
        """提交当前任务剧本的下一条异步动作。"""

        return self._coordinator.dispatch_next()

    def handle_interrupt(self, interrupt):
        """把任务动作终局交回协调器统一投影和分流。"""

        return self._coordinator.handle_execution_interrupt(interrupt)


