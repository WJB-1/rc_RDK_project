"""
contracts/ — 导航层契约子包

拆分为 states/events/commands 三文件；此 __init__ 做扁平 re-export，
使 `from .contracts import X` 或 `from .contracts.states import X` 均可。
"""
from .states import AgentState, EdgeTaskStatus, TurnAction, CulvertType, PlanningInfeasibleError
from .events import (
    Pose, OdomUpdate, RoadCondition,
    CrossroadEvent, CulvertEvent, ObstacleEvent, RfidEvent,
)
from .commands import (
    EdgeTask, DirectedStep, MapEdgeDynamic, TurnCommand,
    NavigationState, CulvertReconResult,
    VisionTools, NavigationAgent,
    ActionCommand, ActionFeedback, ActionPlan, CandidateRoute, Goal, GoalContext,
    MapUpdateIntent, NavigationPlan, NavigationPose, RouteQuery, RuntimeMapView,
    RuntimeMapSnapshot,
)
from .ports import ChoreographerPort, EventSink, IActuator, RoutePlannerPort, RuntimeMapWriter

__all__ = [
    # states
    "AgentState", "EdgeTaskStatus", "TurnAction", "CulvertType", "PlanningInfeasibleError",
    # events
    "Pose", "OdomUpdate", "RoadCondition",
    "CrossroadEvent", "CulvertEvent", "ObstacleEvent", "RfidEvent",
    # commands
    "EdgeTask", "DirectedStep", "MapEdgeDynamic", "TurnCommand",
    "NavigationState", "CulvertReconResult",
    "VisionTools", "NavigationAgent",
    "ActionCommand", "ActionFeedback", "ActionPlan", "CandidateRoute", "Goal",
    "GoalContext", "MapUpdateIntent", "NavigationPlan", "NavigationPose",
    "RouteQuery", "RuntimeMapView", "RuntimeMapSnapshot", "ChoreographerPort",
    "EventSink", "IActuator", "RoutePlannerPort", "RuntimeMapWriter",
]
from .commands import (
    ActionCommand, ActionFeedback, ActionPlan, CandidateRoute, Goal, GoalContext,
    MapUpdateIntent, NavigationPlan, NavigationPose, RouteQuery, RuntimeMapView,
    RuntimeMapSnapshot,
)
from .ports import ChoreographerPort, EventSink, IActuator, RoutePlannerPort, RuntimeMapWriter

__all__ = [
    "ActionCommand", "ActionFeedback", "ActionPlan", "CandidateRoute", "Goal",
    "GoalContext", "MapUpdateIntent", "NavigationPlan", "NavigationPose",
    "RouteQuery", "RuntimeMapView", "RuntimeMapSnapshot", "ChoreographerPort", "EventSink", "IActuator",
    "RoutePlannerPort", "RuntimeMapWriter",
]
