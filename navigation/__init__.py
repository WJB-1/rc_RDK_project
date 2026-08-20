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

from .domain.config import (
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
from .domain.topology import MapNode, MapEdge, RaceTrackTopology, get_topology
from .planning.map_oracle import MapOracle
from .domain.line_graph import LineGraph, edge_heading, turn_cost, has_safe_exit
from .state_machine import AgentState, AgentStateMachine
from .simulation.scene.scene import SimScene, SceneGenerationError
from .simulation.scene.scene_generator import generate_scene
from .simulation.virtual_perception.vision_checker import VisionChecker
from .simulation.sim_engine import SimEngine

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
    # 线图
    "LineGraph",
    "edge_heading",
    "turn_cost",
    "has_safe_exit",
    # 状态机
    "AgentState",
    "AgentStateMachine",
    # 仿真
    "SimScene",
    "SceneGenerationError",
    "generate_scene",
    "VisionChecker",
    "SimEngine",
]
