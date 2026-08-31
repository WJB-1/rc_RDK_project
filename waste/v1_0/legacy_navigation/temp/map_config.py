"""
navigation/map_config.py — 向后兼容 re-export 层

物理常数/节点坐标已迁移到 `navigation/domain/config.py`。
本文件保留以维持 `from navigation.map_config import X` 旧路径不变。
"""
from .domain.config import (
    BLOCK_SIZE_MM, BLOCK_HALF_MM, STRAIGHT_SEGMENT_LENGTH_MM,
    NODE_COORDS, NODE_TYPES, NODE_HAS_RFID, MISSION_NODES, JUNCTION_NODES,
    EDGE_DEFAULTS, TUNNEL_EDGE_DEFAULTS, STATE_MACHINE_CONFIG,
    ACTION_TYPES, EXPECTED_YAW,
)

__all__ = [
    "BLOCK_SIZE_MM", "BLOCK_HALF_MM", "STRAIGHT_SEGMENT_LENGTH_MM",
    "NODE_COORDS", "NODE_TYPES", "NODE_HAS_RFID", "MISSION_NODES",
    "JUNCTION_NODES", "EDGE_DEFAULTS", "TUNNEL_EDGE_DEFAULTS",
    "STATE_MACHINE_CONFIG", "ACTION_TYPES", "EXPECTED_YAW",
]
