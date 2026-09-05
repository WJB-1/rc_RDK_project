"""提供与真实执行器同形的仿真路由执行器。"""

from typing import Iterable

from navigation.contracts import ExecutionTargetPort
from navigation.execution.router import RoutedExecutor
from navigation.contracts import ExecutionEnvironment


class SimExecutor(RoutedExecutor):
    """仿真环境执行器，不发送 UART，只把请求交给虚拟目标端口。"""

    def __init__(self, ports: Iterable[ExecutionTargetPort]) -> None:
        """创建固定为 SIMULATION 的仿真路由器。"""

        super().__init__(ExecutionEnvironment.SIMULATION, ports)
