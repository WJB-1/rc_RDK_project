from dataclasses import dataclass
from enum import Enum
from typing import Callable, Iterable, Optional

from ..contracts import Goal, GoalContext, NavigationPlan, RouteQuery
from ..domain.cost_policy import CostPolicy
from ..domain.mission_state import MissionState
from .goal_selector import GoalSelector


class ControlPhase(str, Enum):
    IDLE = "idle"
    EVALUATING = "evaluating"
    EXECUTING = "executing"
    INTERRUPTED = "interrupted"
    FINISHED = "finished"
    FAILED = "failed"


class InterruptReason(str, Enum):
    OBSTACLE = "obstacle"
    PERCEPTION_UPDATE = "perception_update"
    ACTUATOR_FAILURE = "actuator_failure"
    TIMEOUT = "timeout"


@dataclass
class ControlSnapshot:
    phase: ControlPhase = ControlPhase.IDLE
    selected_goal: Optional[Goal] = None
    map_version: int = 0


class ControlLayer:
    """Serialized mission coordinator and the only production planner trigger."""

    def __init__(self, route_planner, choreographer=None, runtime_map=None,
                 mission_state: MissionState | None = None,
                 goal_selector: GoalSelector | None = None,
                 policy: CostPolicy | None = None,
                 pose=None, start_node: str = "START"):
        self.route_planner = route_planner
        self.choreographer = choreographer
        self.runtime_map = runtime_map
        self.mission_state = mission_state or MissionState()
        self.goal_selector = goal_selector or GoalSelector()
        self.policy = policy or CostPolicy.explore()
        self.pose = pose
        self.start_node = start_node
        self.snapshot = ControlSnapshot()
        self._goal_supplier: Callable[[], Iterable[Goal]] = lambda: ()
        self.current_plan: NavigationPlan | None = None

    def set_goal_supplier(self, supplier: Callable[[], Iterable[Goal]]) -> None:
        self._goal_supplier = supplier

    def generate_candidates(self, checkpoint_goals: Iterable[Goal] = (),
                            culvert_goals: Iterable[Goal] = (),
                            now: float = 0.0) -> tuple[Goal, ...]:
        """Build candidates from mission state without selecting one."""
        candidates = []
        for goal in tuple(checkpoint_goals) + tuple(culvert_goals):
            if self.mission_state.is_candidate(goal.goal_id, now):
                candidates.append(goal)
        return tuple(candidates)

    def start(self) -> None:
        self.snapshot.phase = ControlPhase.IDLE

    def handle_event(self, event) -> None:
        kind = getattr(event, "kind", None)
        if kind in {"map_update", "obstacle", "perception_update"}:
            self.snapshot.selected_goal = None
            self.snapshot.phase = ControlPhase.INTERRUPTED

    def tick(self, feedback=None):
        if self.snapshot.phase in {ControlPhase.EXECUTING, ControlPhase.FINISHED}:
            return self.current_plan
        if self.runtime_map is None or self.pose is None:
            return None
        goals = tuple(self._goal_supplier())
        self.snapshot.phase = ControlPhase.EVALUATING
        query = RouteQuery(self.pose, goals, self.runtime_map.snapshot(), self.policy,
                           start_node=self.start_node)
        routes = self.route_planner.evaluate(query)
        selected = self.goal_selector.select(routes, self.policy)
        if selected is None:
            self.snapshot.phase = ControlPhase.FAILED
            return None
        self.snapshot.selected_goal = selected.goal
        self.snapshot.map_version = query.map_view.version
        self.current_plan = NavigationPlan(selected.goal, selected,
                                           goal_context=GoalContext(selected.goal.goal_type,
                                                                    selected.goal.goal_id),
                                           map_version=query.map_view.version)
        if self.choreographer is not None:
            self.choreographer.compile(self.current_plan)
        self.snapshot.phase = ControlPhase.EXECUTING
        return self.current_plan

    def on_plan_finished(self, result=None) -> None:
        self.snapshot.phase = ControlPhase.IDLE
        self.current_plan = None

    def on_plan_interrupted(self, reason: str = "") -> None:
        self.snapshot.phase = ControlPhase.INTERRUPTED
        self.snapshot.selected_goal = None
        self.current_plan = None
