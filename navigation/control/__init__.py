"""
control/ — 协调层（L2）+ 边执行监控

  - orchestrator.py         : StateOrchestrator（宏观调度）
  - task_queue_controller.py: TaskQueueController（边级任务队列，V4.0 内核）
  - edge_executor.py        : EdgeExecutor（边进度监控）
  - observe_cruise.py       : 观察点三段巡航（侧枝骨架，暂不接入主干）

Task 28 第 4 步「收尸」： CruiseStateMachine（旧边级状态机）已被 TaskQueueController
完全替代，门面（state_machine.py）换核后不再引用，文件已删除、re-export 已移除。
"""
from .orchestrator import StateOrchestrator
from .edge_executor import EdgeExecutor, EdgeProgress, EdgeInterrupts
from .observe_cruise import (
    ThreeSegmentCruise, ThreeSegmentPhase, ObserveDecision, ObserveContext, observe_decide,
)

__all__ = [
    "StateOrchestrator",
    "EdgeExecutor", "EdgeProgress", "EdgeInterrupts",
    "ThreeSegmentCruise", "ThreeSegmentPhase",
    "ObserveDecision", "ObserveContext", "observe_decide",
]
