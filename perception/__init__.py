"""
感知层模块 (Perception Layer) — 编排层

调用 vision/ 算法和 hardware/ 驱动，做感知逻辑判断、画布渲染、事件输出。

职责:
- lane_tracker: 串起分割+IPM 的编排器
- perception_adapter: 感知事件 → Agent 回调
- action_interface: Agent 动作 → 协议编码

设计原则:
1. 不直接调用串口或控制逻辑
2. 所有模块需防御性编程，防止丢帧崩溃
3. 深度学习模型放在 vision/models/，视觉算法放在 vision/
4. 硬件驱动放在 hardware/
"""

_IMPORT_ERRORS = []

try:
    from .lane_tracker import LaneTracker
except Exception as e:
    _IMPORT_ERRORS.append(f"lane_tracker: {e}")
    LaneTracker = None

try:
    from .perception_adapter import PerceptionAdapter
except Exception as e:
    _IMPORT_ERRORS.append(f"perception_adapter: {e}")
    PerceptionAdapter = None

try:
    from .action_interface import ActionInterface
except Exception as e:
    _IMPORT_ERRORS.append(f"action_interface: {e}")
    ActionInterface = None

# contracts 是无依赖模块，始终可导入
from .contracts import LaneState, PerceptionFrame

__all__ = [
    'LaneTracker',
    'PerceptionAdapter',
    'ActionInterface',
    'LaneState',
    'PerceptionFrame',
]

