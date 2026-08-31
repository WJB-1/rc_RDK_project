"""
navigation/map_topology.py — 向后兼容 re-export 层

拓扑已迁移到 `navigation/domain/topology.py`。
本文件保留以维持 `from navigation.map_topology import X` 旧路径不变。
"""
from .domain.topology import MapNode, MapEdge, RaceTrackTopology, get_topology

__all__ = ["MapNode", "MapEdge", "RaceTrackTopology", "get_topology"]
