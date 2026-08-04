"""
决策层模块 (Navigation Layer)

负责高层决策和规划:
- map_config: 全局配置与物理常数
- map_topology: 场地拓扑图定义
- map_oracle: 地图真理查询 API (Dijkstra + Held-Karp)
- state_machine: 全局状态机

设计原则:
1. 决策层可以调用感知层接口
2. 通过communication层与下位机通讯
3. 状态机以高频(50Hz)运转，不阻塞
"""

from .map_config import (
    NODE_COORDS,
    NODE_TYPES,
    NODE_HAS_RFID,
    MISSION_NODES,
    JUNCTION_NODES,
    EDGE_DEFAULTS,
    TUNNEL_EDGE_DEFAULTS,
    STATE_MACHINE_CONFIG,
    ACTION_TYPES,
    EXPECTED_YAW,
)
from .map_topology import MapNode, MapEdge, RaceTrackTopology, get_topology
from .map_oracle import MapOracle
from .state_machine import AgentState, AgentStateMachine

__all__ = [
    # 配置
    "NODE_COORDS",
    "NODE_TYPES",
    "NODE_HAS_RFID",
    "MISSION_NODES",
    "JUNCTION_NODES",
    "EDGE_DEFAULTS",
    "TUNNEL_EDGE_DEFAULTS",
    "STATE_MACHINE_CONFIG",
    "ACTION_TYPES",
    "EXPECTED_YAW",
    # 拓扑
    "MapNode",
    "MapEdge",
    "RaceTrackTopology",
    "get_topology",
    # Oracle
    "MapOracle",
    # 状态机
    "AgentState",
    "AgentStateMachine",
]
