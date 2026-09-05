"""出发状态机的业务流程入口。"""


class DepartureFlow:
    """封装出发阶段的编排、执行和首次观察协调。"""

    def __init__(self, coordinator) -> None:
        """保存公共协调器服务，具体动作仍由 Coordinator 统一提交。"""

        self._coordinator = coordinator

    def dispatch_next(self):
        """请求协调器提交出发剧本的下一条动作。"""

        return self._coordinator.dispatch_next()

    def handle_interrupt(self, interrupt):
        """把出发阶段的终局交回协调器统一校验。"""

        return self._coordinator._handle_execution_interrupt_core(interrupt)

