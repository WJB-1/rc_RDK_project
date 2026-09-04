"""脱困状态机的业务流程入口。"""


class EscapeFlow:
    """封装方向分析、侧支试探和倒车恢复协调。"""

    def __init__(self, coordinator) -> None:
        """保存公共协调器服务。"""

        self._coordinator = coordinator

    def assess_and_replan(self):
        """请求协调器执行当前受困分析和恢复分流。"""

        return self._coordinator.replan()

    def try_side(self, side):
        """请求编排器通过协调器启动指定侧支局部剧本。"""

        return self._coordinator.start_junction_escape(side)

    def dispatch_next(self):
        """提交脱困剧本的下一条异步动作。"""

        return self._coordinator.dispatch_next()


