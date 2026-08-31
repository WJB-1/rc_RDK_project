from typing import Iterable, Optional

from ..contracts import CandidateRoute
from ..domain.cost_policy import CostPolicy


class GoalSelector:
    def select(self, routes: Iterable[CandidateRoute], policy: CostPolicy) -> Optional[CandidateRoute]:
        reachable = [route for route in routes if route.reachable]
        if not reachable:
            return None
        return max(reachable, key=lambda route: route.goal.reward - route.route_cost
                   - max(0.0, route.turn_cost) * policy.turn_weight)
