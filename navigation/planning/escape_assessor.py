"""根据完整目标集合评估路口方向是否值得用于脱困。"""

from navigation.domain import AtNode

from .legal_traversal import LegalTraversalRules
from .models import EscapeAssessment, EscapeDirectionAssessment, JunctionPassability, RouteQuery


class EscapeAssessor:
    """只有驶入方向后仍可达完整目标集合时才标记方向值得尝试。"""

    def __init__(self, topology, reachability_analyzer):
        self._topology = topology
        self._reachability_analyzer = reachability_analyzer

    def assess(self, robot_state, map_snapshot, goals, passability, relative_direction):
        """逐方向假设驶入一条合法边，并评估后续目标集合可达性。"""

        if not isinstance(robot_state.location, AtNode):
            raise ValueError("只有 AtNode 才能进行脱困评估")
        results = {
            "forward": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "left": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "right": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
            "backward": EscapeDirectionAssessment(JunctionPassability.ABSENT, False),
        }
        rules = LegalTraversalRules(self._topology, map_snapshot)
        for edge in self._topology.outgoing_cruise_edges(robot_state.location.node_id):
            direction = relative_direction(robot_state.location.node_id, edge, robot_state.world_pose.yaw_deg)
            if direction is None:
                continue
            status = passability(edge, map_snapshot)
            worth_trying = False
            if (
                status not in (JunctionPassability.BLOCKED, JunctionPassability.ABSENT)
                and rules.is_legal(edge, robot_state.location.entry_traversal_id, robot_state.world_pose.yaw_deg)
            ):
                query = RouteQuery(edge.to_junction, edge.traversal_id, robot_state.world_pose.yaw_deg)
                worth_trying = not self._reachability_analyzer.analyze(query, goals, map_snapshot).is_trapped
            results[direction] = EscapeDirectionAssessment(status, worth_trying, edge.traversal_id)
        return EscapeAssessment(**results)
