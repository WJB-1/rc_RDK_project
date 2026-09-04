"""返回出发区状态机的业务流程入口。"""


class ReturnFlow:
    """封装返回 `J_START` 和出发区尾段的协调。"""

    def __init__(self, coordinator) -> None:
        """保存公共协调器服务。"""

        self._coordinator = coordinator

    def plan(self):
        """请求协调器装载返回阶段的规划结果。"""

        return self._coordinator.replan()

    def dispatch_next(self):
        """提交返回剧本的下一条异步动作。"""

        return self._coordinator.dispatch_next()

