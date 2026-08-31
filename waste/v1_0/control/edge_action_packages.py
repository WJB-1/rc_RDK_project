from ..contracts import ActionCommand


def action(action_id: str, kind: str, **parameters) -> ActionCommand:
    return ActionCommand(action_id, kind, tuple(parameters.items()))


def package_for_step(index: int, step) -> list[ActionCommand]:
    prefix = f"edge-{index}"
    actions = [action(f"{prefix}-turn", "turn", direction="straight", to_node=step.to_node)]
    if step.is_tunnel:
        actions.append(action(f"{prefix}-drive", "drive", distance_mm=step.distance_mm))
    elif step.distance_mm <= 500:
        actions.append(action(f"{prefix}-short-drive", "drive", distance_mm=step.distance_mm))
    else:
        actions.extend([
            action(f"{prefix}-observe-drive", "drive_to_observation", distance_mm=500.0),
            action(f"{prefix}-observe", "observe"),
            action(f"{prefix}-junction-drive", "drive", distance_mm=max(0.0, step.distance_mm - 500.0)),
        ])
    return actions
