"""提供与真实执行器同形的仿真路由执行器。"""

from typing import Iterable

from navigation.contracts import ExecutionTargetPort
from navigation.execution.router import RoutedExecutor
from navigation.contracts import ExecutionEnvironment


class SimExecutor(RoutedExecutor):
    """仿真环境执行器，不发送 UART，只把请求交给虚拟目标端口。"""

    def __init__(self, ports: Iterable[ExecutionTargetPort]) -> None:
        """创建固定为 SIMULATION 的仿真路由器。"""

        self._simulation_ports = tuple(ports)
        super().__init__(ExecutionEnvironment.SIMULATION, self._simulation_ports)

    def complete_next(self) -> bool:
        """推进装配端口中的一条最早可完成请求。"""

        for port in self._simulation_ports:
            if getattr(port, "has_pending", lambda: False)():
                port.complete_next()
                return True
        return False

    def has_pending(self) -> bool:
        """返回是否存在待推进的虚拟终局。"""

        return any(getattr(port, "has_pending", lambda: False)() for port in self._simulation_ports)
