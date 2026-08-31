import heapq
import math
from typing import Optional

from ..domain.line_graph import edge_heading, turn_cost
from ..domain.topology import RaceTrackTopology, get_topology


class DirectedDijkstra:
    """Runtime shortest-path tool; it has no TSP or mission-ordering logic."""

    def __init__(self, topology: RaceTrackTopology | None = None):
        self.topology = topology or get_topology()

    def shortest_path(self, start: str, goal: str, blocked_edges=frozenset(),
                      entry_heading: Optional[float] = None):
        if start not in self.topology.nodes or goal not in self.topology.nodes:
            return math.inf, None
        initial = (start, None)
        distances = {initial: 0.0}
        previous = {}
        queue = [(0.0, start, None)]
        reached = None
        blocked = frozenset(blocked_edges)
        while queue:
            cost, node, prior = heapq.heappop(queue)
            state = (node, prior)
            if cost != distances.get(state):
                continue
            if node == goal:
                reached = state
                break
            for edge in self.topology.get_neighbors(node):
                if edge.edge_id in blocked:
                    continue
                target = edge.other(node)
                incoming = entry_heading if prior is None else edge_heading(self.topology, prior, node)
                outgoing = edge_heading(self.topology, node, target)
                turn = 0.0 if incoming is None else turn_cost(incoming, outgoing)
                if math.isinf(turn):
                    continue
                target_state = (target, node)
                target_cost = cost + edge.distance_mm + turn
                if target_cost < distances.get(target_state, math.inf):
                    distances[target_state] = target_cost
                    previous[target_state] = state
                    heapq.heappush(queue, (target_cost, target, node))
        if reached is None:
            return math.inf, None
        path = []
        state = reached
        while state is not None:
            path.append(state[0])
            state = previous.get(state)
        return distances[reached], list(reversed(path))
