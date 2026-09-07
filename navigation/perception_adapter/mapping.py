"""实现相对视觉区域到静态巡航边的只读映射。"""

import math
from typing import Optional

from navigation.domain import AtNode, OnCruiseEdge, RobotState, TrackTopology


def map_region_to_traversal(robot_state: RobotState, topology: TrackTopology, region: str) -> Optional[str]:
    """按机器人当前位置、朝向和静态拓扑解析视觉区域对应的有向边。"""
    if isinstance(robot_state.location, OnCruiseEdge) and region in ("CURRENT_LANE", "FORWARD_BRANCH", "FORWARD"):
        return robot_state.location.traversal_id
    if not isinstance(robot_state.location, AtNode):
        return None
    candidates = topology.outgoing_cruise_edges(robot_state.location.node_id)
    if region in ("LEFT_BRANCH", "LEFT"):
        predicate = lambda difference: 45.0 <= difference <= 135.0
    elif region in ("RIGHT_BRANCH", "RIGHT"):
        predicate = lambda difference: -135.0 <= difference <= -45.0
    elif region in ("FORWARD_BRANCH", "FORWARD", "CURRENT_LANE"):
        predicate = lambda difference: abs(difference) <= 45.0
    else:
        return None
    origin = topology.get_node(robot_state.location.node_id)
    ranked = []
    for edge in candidates:
        target = topology.get_node(edge.to_junction)
        road_heading = math.degrees(math.atan2(target.y_mm - origin.y_mm, target.x_mm - origin.x_mm))
        difference = (road_heading - robot_state.heading_deg + 180.0) % 360.0 - 180.0
        if predicate(difference):
            ranked.append((abs(difference), edge.traversal_id))
    return min(ranked)[1] if ranked else None
