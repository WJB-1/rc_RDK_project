from dataclasses import replace
from uuid import uuid4

from ..contracts import ActionFeedback, ActionPlan
from .edge_action_packages import package_for_step


class Choreographer:
    def __init__(self):
        self._active: ActionPlan | None = None
        self._cursor = 0

    def compile(self, plan):
        commands = []
        for index, step in enumerate(plan.route.steps):
            commands.extend(package_for_step(index, step))
        result = ActionPlan(str(uuid4()), tuple(commands), plan.goal_context, plan.map_version)
        self._active = result
        self._cursor = 0
        return result

    def step(self, feedback: ActionFeedback):
        if self._active is None:
            return None
        if self._cursor >= len(self._active.actions):
            return None
        current = self._active.actions[self._cursor]
        if feedback.action_id != current.action_id:
            return self._active
        if feedback.status in {"succeeded", "completed", "success"}:
            self._cursor += 1
        elif feedback.status in {"failed", "cancelled", "interrupted"}:
            self._active = None
            return None
        return self._active if self._cursor < len(self._active.actions) else None

    def cancel(self, reason: str = "") -> None:
        self._active = None
        self._cursor = 0

    @property
    def current_action(self):
        if self._active is None or self._cursor >= len(self._active.actions):
            return None
        return self._active.actions[self._cursor]

    @property
    def active_plan(self):
        return self._active
