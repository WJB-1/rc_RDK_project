"""提供不发送 UART 的导航仿真执行器。"""

# 导出仿真执行器，虚拟目标端口由 SimulationRunner 装配并推进事件。
from .executor import SimExecutor

__all__ = ("SimExecutor",)
