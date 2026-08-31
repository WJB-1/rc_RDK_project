from ..contracts import ActionCommand, ActionFeedback


class VirtualActuator:
    """Stateless simulation actuator for the new ActionCommand contract."""

    def send(self, command: ActionCommand) -> ActionFeedback:
        return ActionFeedback(command.action_id, "succeeded")

    def cancel(self, reason: str = "") -> None:
        return None
