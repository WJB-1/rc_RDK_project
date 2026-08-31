"""
contracts/states.py — 导航层状态与动作枚举

跨层共享的状态枚举（AgentState / EdgeTaskStatus / TurnAction / CulvertType）。
"""
from enum import Enum, auto


class AgentState(Enum):
    """Agent 状态枚举 — 与 state_machine.py 同步"""
    IDLE = auto()
    GLOBAL_PLANNING = auto()
    EDGE_EXECUTING = auto()
    APPROACHING = auto()
    TURNING = auto()
    NODE_ARRIVAL = auto()
    CULVERT_RECON = auto()
    OBSTACLE_STOP = auto()
    BACKTRACK = auto()
    FAILED = auto()
    FINISHED = auto()


class EdgeTaskStatus(Enum):
    """边级任务状态"""
    PENDING = "pending"
    EXECUTING = "executing"
    DONE = "done"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


class TurnAction(Enum):
    """转向动作枚举"""
    STRAIGHT = "straight"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    UTURN = "uturn"
    STOP = "stop"


class CulvertType(Enum):
    """涵洞检测类型"""
    FRONT = "front"      # 正前方涵洞入口
    SIDE = "side"        # 侧面涵洞（路口/拐角处）


class PlanningInfeasibleError(Exception):
    """
    规划不可行异常（Task 28 后续：180° 掉头 fail fast）。

    语义：路径规划产出了非法结果（如「原地 180° 掉头」指令）。这是硬逻辑错误
    而非可驾驶动作——上层调用方（`_determine_turn` / `_dijkstra` 调用链）漏传
    朝向参数或规划层生成了非法折返，正确地发生即说明代码有 bug，应直接抛错死机，
    绝不静默降级为 STOP / 伪造转向。
    """
    def __init__(self, message: str = "规划不可行"):
        self.message = message
        super().__init__(message)
