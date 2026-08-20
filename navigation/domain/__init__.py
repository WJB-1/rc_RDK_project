"""
domain/ — 导航领域模型：静态拓扑 + 线图 + 物理常数

拆分后的归属：
  - config.py      : 物理常数 / 节点坐标 / RFID 配置（原 map_config）
  - topology.py    : RaceTrackTopology / MapNode / MapEdge（原 map_topology）
  - line_graph.py  : LineGraph / 有向边 + 转向代价（原 line_graph）
  - runtime_map.py : RuntimeMap（运行时事实，后续任务 D-11 落地）
"""
from .config import (
    NODE_COORDS, NODE_TYPES, NODE_HAS_RFID, MISSION_NODES, JUNCTION_NODES,
    EDGE_DEFAULTS, TUNNEL_EDGE_DEFAULTS, STATE_MACHINE_CONFIG,
    ACTION_TYPES, EXPECTED_YAW,
)
from .topology import MapNode, MapEdge, RaceTrackTopology, get_topology
from .line_graph import LineGraph, edge_heading, turn_cost, has_safe_exit
from .runtime_map import RuntimeMap, END_MIDDLE

__all__ = [
    "NODE_COORDS", "NODE_TYPES", "NODE_HAS_RFID", "MISSION_NODES",
    "JUNCTION_NODES", "EDGE_DEFAULTS", "TUNNEL_EDGE_DEFAULTS",
    "STATE_MACHINE_CONFIG", "ACTION_TYPES", "EXPECTED_YAW",
    "MapNode", "MapEdge", "RaceTrackTopology", "get_topology",
    "LineGraph", "edge_heading", "turn_cost", "has_safe_exit",
    "RuntimeMap", "END_MIDDLE",
]
