"""Offline TSP baseline for reporting and comparison only."""
from itertools import permutations


def solve_visit_order(dijkstra, start_node: str, targets: tuple[str, ...], blocked_edges=frozenset()):
    """Return the cheapest target order; never use this from runtime navigation."""
    best_cost, best_order = float("inf"), ()
    for order in permutations(targets):
        cursor, cost = start_node, 0.0
        for target in order:
            distance, path = dijkstra.shortest_path(cursor, target, blocked_edges)
            if path is None:
                cost = float("inf")
                break
            cost += distance
            cursor = target
        if cost < best_cost:
            best_cost, best_order = cost, order
    return best_cost, best_order
