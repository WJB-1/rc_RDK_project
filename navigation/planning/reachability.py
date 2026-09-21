"""执行规划前的合法可达性分析。"""

from dataclasses import dataclass
from collections import deque
from typing import Tuple

from navigation.domain import Goal, TrackTopology

from .models import RouteQuery
from .legal_traversal import LegalTraversalRules


@dataclass(frozen=True)
class ReachabilityReport:
    """描述当前完整目标集合是否存在至少一个合法可达目标。"""

    reachable_goal_ids: Tuple[str, ...]

    @property
    def is_trapped(self) -> bool:
        """没有任何目标可达即按当前业务阶段定义为受困。"""

        return not self.reachable_goal_ids


class ReachabilityAnalyzer:
    """用共享合法边规则分析任务态或返回态目标集合的可达性。"""

    def __init__(self, topology: TrackTopology):
        self._topology = topology

    def analyze(self, query: RouteQuery, goals: Tuple[Goal, ...], map_snapshot) -> ReachabilityReport:
        """执行一次 BFS 可达性搜索并返回可达目标标识。"""

        start_state = (query.start_node_id, query.entry_traversal_id)
        reachable_states = {start_state}
        queue = deque((start_state,))
        rules = LegalTraversalRules(self._topology, map_snapshot)
        while queue:
            node_id, entry_traversal_id = queue.popleft()
            for edge in rules.outgoing(node_id, entry_traversal_id, query.heading_deg):
                next_state = (edge.to_junction, edge.traversal_id)
                if next_state in reachable_states:
                    continue
                reachable_states.add(next_state)
                queue.append(next_state)
        reachable = []
        for goal in goals:
            for node_id, entry_traversal_id in reachable_states:
                if node_id != goal.arrival_node_id:
                    continue
                if goal.required_final_traversal_id and entry_traversal_id != goal.required_final_traversal_id:
                    continue
                if self._entered_via_approach_edge(entry_traversal_id, goal):
                    continue
                reachable.append(goal.goal_id)
                break
        return ReachabilityReport(tuple(sorted(reachable)))

    def _entered_via_approach_edge(self, entry_traversal_id, goal: Goal) -> bool:
        """拒绝沿涵洞任务道路驶入终点的状态，保持与正常路径终点约束一致。"""

        if goal.approach_edge_id is None or entry_traversal_id is None:
            return False
        from_junction, to_junction = entry_traversal_id.split("->", 1)
        edge = self._topology.get_cruise_edge(from_junction, to_junction)
        return goal.approach_edge_id in edge.physical_edge_ids
