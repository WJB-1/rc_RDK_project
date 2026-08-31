from typing import Protocol, Sequence

from .commands import (ActionCommand, ActionFeedback, ActionPlan, CandidateRoute,
                       NavigationPlan, RouteQuery, MapUpdateIntent)


class RoutePlannerPort(Protocol):
    def evaluate(self, query: RouteQuery) -> Sequence[CandidateRoute]: ...


class ChoreographerPort(Protocol):
    def compile(self, plan: NavigationPlan) -> ActionPlan: ...
    def step(self, feedback: ActionFeedback) -> ActionPlan | None: ...
    def cancel(self, reason: str = "") -> None: ...


class EventSink(Protocol):
    def publish(self, event: object) -> None: ...


class RuntimeMapWriter(Protocol):
    def apply(self, intent: MapUpdateIntent) -> object: ...


class IActuator(Protocol):
    def send(self, command: ActionCommand) -> ActionFeedback | None: ...
    def cancel(self, reason: str = "") -> None: ...
