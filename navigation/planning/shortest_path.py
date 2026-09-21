"""提供一次全局 Dijkstra 搜索和路线重建。"""

import heapq
from typing import Dict, Optional, Tuple

from navigation.domain import CruiseEdge, RuntimeMapSnapshot, TrackTopology

from .legal_traversal import LegalTraversalRules


SearchState = Tuple[str, Optional[str]]


class GlobalShortestPath:
    """在当前地图快照上执行一次有向全局最短路搜索。"""

    def __init__(self, topology: TrackTopology, map_snapshot: RuntimeMapSnapshot):
        self._topology = topology
        self._map_snapshot = map_snapshot

    def search(self, start_state: SearchState, heading_deg: float) -> Tuple[Dict[SearchState, float], Dict[SearchState, Tuple[SearchState, CruiseEdge]]]:
        """返回所有合法状态的距离与前驱，供多目标和探索候选共同消费。"""

        distances: Dict[SearchState, float] = {start_state: 0.0}
        predecessors: Dict[SearchState, Tuple[SearchState, CruiseEdge]] = {}
        queue = [(0.0, 0, start_state)]
        sequence = 1
        rules = LegalTraversalRules(self._topology, self._map_snapshot)
        while queue:
            distance_mm, _, state = heapq.heappop(queue)
            if distance_mm != distances[state]:
                continue
            node_id, entry_traversal_id = state
            for edge in rules.outgoing(node_id, entry_traversal_id, heading_deg):
                next_state = (edge.to_junction, edge.traversal_id)
                next_distance = distance_mm + edge.length_mm
                if next_state in distances and distances[next_state] <= next_distance:
                    continue
                distances[next_state] = next_distance
                predecessors[next_state] = (state, edge)
                heapq.heappush(queue, (next_distance, sequence, next_state))
                sequence += 1
        return distances, predecessors
