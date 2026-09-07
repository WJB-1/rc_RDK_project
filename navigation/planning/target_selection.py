"""根据只读任务和地图快照选择正常规划目标。"""

from typing import Optional, Tuple

from navigation.domain import Goal, GoalKind, Task, TrackTopology

from .goal_deriver import GoalDeriver
from .models import PlanningPhase, RoutePlanOutcome


class TargetSelectionResult:
    """封装目标候选、探索边和显式拒绝结果。"""

    def __init__(self, outcome, goals: Tuple[Goal, ...] = (), reason: Optional[str] = None):
        self.outcome = outcome
        self.goals = goals
        self.reason = reason


class TargetSelector:
    """只读选择任务目标、探索目标或返回目标，不改变任何任务状态。"""

    def __init__(self, topology: TrackTopology):
        self._topology = topology
        self._goal_deriver = GoalDeriver(topology)

    def select(self, phase: PlanningPhase, pending_tasks: Tuple[Task, ...], mission_finished: bool, map_snapshot) -> TargetSelectionResult:
        """按阶段选择候选；任务已完成时只返回拒绝，不代替协调器切换状态。"""

        if phase is PlanningPhase.RETURNING:
            return TargetSelectionResult(None, (Goal("return@J_START", GoalKind.RETURN_TO_START, "J_START"),))
        if pending_tasks:
            goals = tuple(goal for task in pending_tasks for goal in self._goal_deriver.derive(task))
            return TargetSelectionResult(None, goals, "待办任务目标")
        if mission_finished:
            return TargetSelectionResult(RoutePlanOutcome.TASKS_COMPLETED, reason="任务已完成，任务态拒绝继续正常规划")
        goals = []
        for node_id in self._topology.node_ids():
            for edge in self._topology.outgoing_cruise_edges(node_id):
                external_edge_ids = tuple(
                    edge_id
                    for edge_id in edge.physical_edge_ids
                    if self._topology.get_physical_edge(edge_id).road_kind != "INTERNAL"
                )
                if any(edge_id in map_snapshot.blocked_edge_ids for edge_id in external_edge_ids):
                    continue
                if any(edge_id in map_snapshot.discovered_culvert_edge_ids for edge_id in external_edge_ids):
                    continue
                if external_edge_ids and all(edge_id in map_snapshot.confirmed_no_culvert_edge_ids for edge_id in external_edge_ids):
                    continue
                goals.append(
                    Goal(
                        "explore@{}".format(edge.traversal_id),
                        GoalKind.TASK_ARRIVAL,
                        edge.to_junction,
                        required_final_traversal_id=edge.traversal_id,
                    )
                )
        goals.sort(key=lambda goal: goal.goal_id)
        return TargetSelectionResult(None, tuple(goals), "最近未探索边候选")
