"""提供不发送 UART 的导航仿真执行器。"""

# 导出仿真执行器，虚拟目标端口由 SimulationRunner 装配并推进事件。
from .executor import HybridExecutor, SimExecutor
from .ports import SimMotionPort, SimPerceptionPort, SimTaskPort
from .snapshot import SimWorldSnapshot, SimMotionResult, SimTaskResult, SimulationSnapshot
from .world import SimWorld
from .runner import HardwareMotionSimulationRunner, SimulationRunner
from .composition import build_hardware_motion_simulation_runner, build_simulation_runner

__all__ = ("SimExecutor", "HybridExecutor", "SimMotionPort", "SimPerceptionPort", "SimTaskPort",
           "SimWorld", "SimWorldSnapshot", "SimMotionResult", "SimTaskResult", "SimulationSnapshot", "SimulationRunner",
           "HardwareMotionSimulationRunner", "build_simulation_runner", "build_hardware_motion_simulation_runner")
