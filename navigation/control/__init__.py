"""
control/ — 协调层（L2）+ 边执行监控

  - orchestrator.py         : StateOrchestrator（宏观调度）
  - cruise_state_machine.py : CruiseStateMachine（边级状态转移）
  - edge_executor.py        : EdgeExecutor（边进度监控）
  - observe_cruise.py       : 观察点三段巡航（侧枝骨架，暂不接入主干）
"""
from .orchestrator import StateOrchestrator
from .cruise_state_machine import CruiseStateMachine
from .edge_executor import EdgeExecutor, EdgeProgress, EdgeInterrupts
from .observe_cruise import (
    ThreeSegmentCruise, ThreeSegmentPhase, ObserveDecision, ObserveContext, observe_decide,
)

__all__ = [
    "StateOrchestrator", "CruiseStateMachine",
    "EdgeExecutor", "EdgeProgress", "EdgeInterrupts",
    "ThreeSegmentCruise", "ThreeSegmentPhase",
    "ObserveDecision", "ObserveContext", "observe_decide",
]
