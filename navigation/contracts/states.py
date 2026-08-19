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
