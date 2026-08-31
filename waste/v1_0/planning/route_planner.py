from typing import Sequence

from ..contracts import CandidateRoute, RouteQuery
from ..contracts.commands import DirectedStep
from .directed_dijkstra import DirectedDijkstra


class RoutePlanner:
    """Evaluate every candidate independently; selection remains in control."""

    def __init__(self, dijkstra: DirectedDijkstra):
        self.dijkstra = dijkstra

    def evaluate(self, query: RouteQuery) -> Sequence[CandidateRoute]:
        results = []
        for goal in query.candidates:
            target = goal.node_name
            if not target and goal.edge_id is not None:
                try:
                    edge = self.dijkstra.topology.get_edge_by_id(goal.edge_id)
                    target = edge.node_a
                except KeyError:
                    target = ""
            if not target or target not in self.dijkstra.topology.nodes:
                results.append(CandidateRoute(goal, False, failure_reason="unknown_target"))
                continue
            _, path = self.dijkstra.shortest_path(query.start_node, target,
                                                   query.map_view.blocked_edges)
            if not path:
                results.append(CandidateRoute(goal, False, failure_reason="unreachable"))
                continue
            steps = []
            distance = 0.0
            for source, destination in zip(path, path[1:]):
                edge = self.dijkstra.topology.get_edge(source, destination)
                distance += edge.distance_mm
                from ..domain.line_graph import edge_heading
                steps.append(DirectedStep(source, destination,
                                          edge_heading(self.dijkstra.topology, source, destination),
                                          edge.distance_mm, edge.is_tunnel))
            cost = query.policy.route_cost(distance)
            results.append(CandidateRoute(goal, True, tuple(steps), cost, distance))
        return tuple(results)
