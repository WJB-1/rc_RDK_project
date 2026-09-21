"""Navigation 2.0 的纯编排公共入口。

谁调用：后续 `Coordinator` 在取得路线或恢复计划后创建并推进流程。
谁响应：本包只提供 `Choreographer` 和它读取的 `MotionProfile`。
输入输出：输入规划层语义计划与只读状态查询；输出不可变流程剧本和类型化动作。
状态影响：不直接提交执行器、不写入机器人状态、动态地图或任务状态。
"""

from .choreographer import Choreographer
from .profile import (
    BODY_HALF_LENGTH_MM,
    DEPARTURE_FORWARD_MM,
    OBSERVATION_ZONE_FROM_CENTER_MM,
    RETRACE_ANCHOR_FROM_CENTER_MM,
    TAIL_ANCHOR_FROM_CENTER_MM,
    TURN_WINDOW_FROM_CENTER_MM,
)

__all__ = (
    "Choreographer",
    "OBSERVATION_ZONE_FROM_CENTER_MM",
    "TURN_WINDOW_FROM_CENTER_MM",
    "TAIL_ANCHOR_FROM_CENTER_MM",
    "RETRACE_ANCHOR_FROM_CENTER_MM",
    "BODY_HALF_LENGTH_MM",
    "DEPARTURE_FORWARD_MM",
)
