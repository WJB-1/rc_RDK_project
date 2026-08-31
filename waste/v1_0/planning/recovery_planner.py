from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryPlan:
    steps: tuple[dict, ...]
    safe_pose: object = None


class RecoveryPlanner:
    def plan(self, blocked_edge: int, current_pose, reverse_distance_mm: float = 300.0) -> RecoveryPlan:
        return RecoveryPlan((
            {"kind": "reverse", "edge_id": blocked_edge, "distance_mm": max(0.0, reverse_distance_mm)},
            {"kind": "reverse_turn", "edge_id": blocked_edge},
            {"kind": "forward_resume", "edge_id": blocked_edge},
        ), current_pose)
