"""集中定义正常规划与可达性分析共用的合法巡航规则。"""

import math
from typing import Optional

from navigation.domain import CruiseEdge, EdgeKnowledgeStatus, RuntimeMapSnapshot, TrackTopology


class LegalTraversalRules:
    """只读判断一条巡航边能否作为正常路线的下一步。"""

    def __init__(self, topology: TrackTopology, map_snapshot: RuntimeMapSnapshot):
        self._topology = topology
        self._map_snapshot = map_snapshot

    def outgoing(self, node_id: str, entry_traversal_id: Optional[str], heading_deg: float):
        """按拓扑既定顺序返回所有合法的下一条巡航边。"""

        for edge in self._topology.outgoing_cruise_edges(node_id):
            if self.is_legal(edge, entry_traversal_id, heading_deg):
                yield edge

    def is_legal(self, edge: CruiseEdge, entry_traversal_id: Optional[str], heading_deg: float) -> bool:
        """检查阻塞、立即反向和初始原地掉头约束。"""

        if any(self._edge_status(edge_id) is EdgeKnowledgeStatus.BLOCKED for edge_id in edge.physical_edge_ids):
            return False
        if self._is_reverse(entry_traversal_id, edge.traversal_id):
            return False
        if entry_traversal_id is None and self._is_initial_uturn(edge, heading_deg):
            return False
        return True

    def _edge_status(self, edge_id: str):
        """通过 RuntimeMapSnapshot 的状态查询接口读取最新边状态。"""

        status_method = getattr(self._map_snapshot, "edge_status", None)
        if status_method is not None:
            return status_method(edge_id)
        return (
            EdgeKnowledgeStatus.BLOCKED
            if edge_id in self._map_snapshot.blocked_edge_ids
            else EdgeKnowledgeStatus.UNKNOWN
        )

    @staticmethod
    def _is_reverse(entry_traversal_id: Optional[str], next_traversal_id: str) -> bool:
        if entry_traversal_id is None:
            return False
        from_node_id, to_node_id = entry_traversal_id.split("->", 1)
        return next_traversal_id == "{}->{}".format(to_node_id, from_node_id)

    def _is_initial_uturn(self, edge: CruiseEdge, heading_deg: float) -> bool:
        start = self._topology.get_node(edge.from_junction)
        end = self._topology.get_node(edge.to_junction)
        road_heading = math.degrees(math.atan2(end.y_mm - start.y_mm, end.x_mm - start.x_mm))
        difference = abs((road_heading - heading_deg + 180.0) % 360.0 - 180.0)
        return abs(difference - 180.0) < 0.000001
