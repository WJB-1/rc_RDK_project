"""
navigation/map_topology.py - 赛道全局拓扑与节点物理坐标定义

基于《物理地图建模与Agent接口规范.md》第1、2节实现。
提供：
- 20个节点的物理坐标、类型、RFID标记
- 无向边连接关系与物理属性
- 辅助工具函数（坐标计算、欧氏距离等）

拓扑结构:

       START (0,0)
         |
         | 200mm
         ▼
    J_START (0,200)  ← T型路口
    ───┬────────┬───
  N1(-400)  N12(400)   ← 第一排巡逻点

    N2(-400)  N11(400)
    ───┬──[T1隧道]──┬───
    N3(-400)  N10(400)
    ───┬──[T2隧道]──┬───
    N4(-400)   N9(400)
    ───┬──[T3隧道]──┬───
    N5  ── N6==N7 ── N8  ← 底部(含隧道段)
"""

import math
from typing import Dict, List, Tuple

from .map_config import (
    NODE_COORDS,
    NODE_TYPES,
    NODE_HAS_RFID,
    MISSION_NODES,
    JUNCTION_NODES,
    EDGE_DEFAULTS,
    TUNNEL_EDGE_DEFAULTS,
    STRAIGHT_SEGMENT_LENGTH_MM,
    START_DEPTH_MM,
    HALF_TRACK_SPAN_MM,
    LANE_HALF_WIDTH_MM,
)


class MapNode:
    """地图节点数据结构"""

    def __init__(self, name: str, x_mm: float, y_mm: float, node_type: str, has_rfid: bool):
        self.name = name
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.node_type = node_type       # "base" | "mission" | "junction"
        self.has_rfid = has_rfid
        self.is_visited = False          # 打卡状态，由 Agent 状态机维护

    def __repr__(self) -> str:
        return (
            f"MapNode({self.name}, x={self.x_mm}, y={self.y_mm}, "
            f"type={self.node_type}, rfid={self.has_rfid}, visited={self.is_visited})"
        )

    def distance_to(self, other: "MapNode") -> float:
        """计算与另一节点的欧氏距离 (mm)"""
        return math.hypot(self.x_mm - other.x_mm, self.y_mm - other.y_mm)

    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "x": self.x_mm,
            "y": self.y_mm,
            "type": self.node_type,
            "has_rfid": self.has_rfid,
            "is_visited": self.is_visited,
        }


class MapEdge:
    """
    地图边数据结构

    静态属性（编译期确定）: node_a/b, distance_mm, is_tunnel, speed_limit_ms
    动态属性（运行时更新）: is_blocked, has_culvert, visit_count, edge_id
    """

    _next_edge_id = 0

    def __init__(self, node_a: str, node_b: str, distance_mm: float,
                 is_tunnel: bool = False, has_culvert: bool = False,
                 speed_limit_ms: float = None):
        self.edge_id = MapEdge._next_edge_id
        MapEdge._next_edge_id += 1
        self.node_a = node_a
        self.node_b = node_b
        self.distance_mm = distance_mm
        self.is_tunnel = is_tunnel
        self.has_culvert = has_culvert
        self.speed_limit_ms = speed_limit_ms or (
            TUNNEL_EDGE_DEFAULTS["speed_limit_ms"] if is_tunnel else EDGE_DEFAULTS["speed_limit_ms"]
        )
        # 运行时动态属性
        self.is_blocked: bool = False
        self.visit_count: int = 0

    def __repr__(self) -> str:
        return (
            f"MapEdge({self.node_a} <-> {self.node_b}, d={self.distance_mm}mm, "
            f"tunnel={self.is_tunnel}, blocked={self.is_blocked}, visits={self.visit_count})"
        )

    def to_dict(self) -> Dict:
        return {
            "edge_id": self.edge_id,
            "node_a": self.node_a,
            "node_b": self.node_b,
            "distance_mm": self.distance_mm,
            "is_tunnel": self.is_tunnel,
            "has_culvert": self.has_culvert,
            "is_blocked": self.is_blocked,
            "visit_count": self.visit_count,
            "speed_limit_ms": self.speed_limit_ms,
        }

    def other(self, node_name: str) -> str:
        """给定一端节点名，返回另一端"""
        if node_name == self.node_a:
            return self.node_b
        if node_name == self.node_b:
            return self.node_a
        raise ValueError(f"节点 {node_name} 不在边 {self.node_a}-{self.node_b} 上")


class RaceTrackTopology:
    """
    比赛赛道拓扑模型 (单例友好，可直接实例化使用)

    节点命名规范：
    - START: 出发区原点
    - J_START: START 与赛道 N1-N12 交点 (T型路口)
    - N1~N12: 必到任务点 (左列奇数，右列偶数，编号从上到下)
    - T1_L/T1_R, T2_L/T2_R, T3_L/T3_R: 三行隧道的辅助路口节点
    """

    def __init__(self):
        self.nodes: Dict[str, MapNode] = {}
        self.edges: List[MapEdge] = []
        self._adj: Dict[str, List[MapEdge]] = {}
        self._build_nodes()
        self._build_edges()

    # ------------------------------------------
    # 构建节点
    # ------------------------------------------
    def _build_nodes(self):
        for name, coord in NODE_COORDS.items():
            self.nodes[name] = MapNode(
                name=name,
                x_mm=coord["x"],
                y_mm=coord["y"],
                node_type=NODE_TYPES[name],
                has_rfid=NODE_HAS_RFID[name],
            )
            self._adj[name] = []

    # ------------------------------------------
    # 构建边（严格按照规范拓扑图）
    # ------------------------------------------
    def _build_edges(self):
        """
        拓扑连接关系（无向边）：

        START (0,0)
         |
         | 200mm
         ▼
        J_START (0,200)
        ───┬───────────┬───
        N1(-400)    N12(400)

        N1 ─ N2 == T1_L == T1_R == N11 ─ N10 ─ N9 ─ N8
             │                                       │    │
            N3 ─ ... 右侧对称 ...                  N4   N7
                                                   │    │
                                                  N5 ─ N6 ─ N7 (底部)
        """
        # 所有"车道边"统一 800mm
        # 隧道: T1_L↔T1_R, T2_L↔T2_R, T3_L↔T3_R, N6↔N7
        LO = STRAIGHT_SEGMENT_LENGTH_MM  # 800mm

        edge_specs = [
            # START → J_START (启动桥 200mm)
            ("START",  "J_START", START_DEPTH_MM, False, False),

            # J_START → N1/N12 (400mm)
            ("J_START", "N1",     HALF_TRACK_SPAN_MM, False, False),
            ("J_START", "N12",    HALF_TRACK_SPAN_MM, False, False),

            # N1-N12 顶部水平 (800mm)
            ("N1",    "N12",     HALF_TRACK_SPAN_MM * 2, False, False),

            # 左内纵列 (X=-400): N1→T1_L→T2_L→T3_L→N6, 每段 800mm
            ("N1",    "T1_L",   LO, False, False),
            ("T1_L",  "T2_L",   LO, False, False),
            ("T2_L",  "T3_L",   LO, False, False),
            ("T3_L",  "N6",     LO, False, False),

            # 右内纵列 (X=+400): N12→T1_R→T2_R→T3_R→N7, 每段 800mm
            ("N12",   "T1_R",   LO, False, False),
            ("T1_R",  "T2_R",   LO, False, False),
            ("T2_R",  "T3_R",   LO, False, False),
            ("T3_R",  "N7",     LO, False, False),

            # 左外侧纵列 (X=-1200): N2→N3→N4→N5, 每段 800mm
            ("N2",    "N3",     LO, False, False),
            ("N3",    "N4",     LO, False, False),
            ("N4",    "N5",     LO, False, False),

            # 右外侧纵列 (X=+1200): N11→N10→N9→N8
            ("N11",   "N10",    LO, False, False),
            ("N10",   "N9",     LO, False, False),
            ("N9",    "N8",     LO, False, False),

            # 内外纵列横向连接 (X=-1200 ↔ X=-400): N2↔T1_L, N3↔T2_L, N4↔T3_L
            ("N2",    "T1_L",   LO, False, False),
            ("N3",    "T2_L",   LO, False, False),
            ("N4",    "T3_L",   LO, False, False),

            # 内外纵列横向连接 (X=+1200 ↔ X=+400): T1_R↔N11, T2_R↔N10, T3_R↔N9
            ("T1_R",  "N11",    LO, False, False),
            ("T2_R",  "N10",    LO, False, False),
            ("T3_R",  "N9",     LO, False, False),

            # 隧道: T_L↔T_R = 800mm
            ("T1_L",  "T1_R",   LO, True,  False),
            ("T2_L",  "T2_R",   LO, True,  False),
            ("T3_L",  "T3_R",   LO, True,  False),

            # 底部: N5(-1200,3400)→N6(-400,3400) 800mm → N6→N7 隧道 → N7(400,3400)→N8(1200,3400) 800mm
            ("N5",    "N6",     LO,  False, False),
            ("N6",    "N7",     LO,  True,  False),
            ("N7",    "N8",     LO,  False, False),
        ]

        for a, b, dist, is_tunnel, has_culvert in edge_specs:
            self._add_edge(a, b, dist, is_tunnel, has_culvert)

    def _add_edge(self, a: str, b: str, dist: float, is_tunnel: bool, has_culvert: bool):
        edge = MapEdge(a, b, dist, is_tunnel, has_culvert)
        self.edges.append(edge)
        self._adj[a].append(edge)
        self._adj[b].append(edge)

    # ------------------------------------------
    # 查询接口
    # ------------------------------------------
    def get_node(self, name: str) -> MapNode:
        if name not in self.nodes:
            raise KeyError(f"节点 {name} 不存在")
        return self.nodes[name]

    def get_neighbors(self, node_name: str) -> List[MapEdge]:
        """返回与某节点相连的所有边"""
        if node_name not in self._adj:
            return []
        return self._adj[node_name][:]

    def get_neighbor_names(self, node_name: str) -> List[str]:
        """返回某节点的邻居节点名称列表"""
        return [e.other(node_name) for e in self.get_neighbors(node_name)]

    def get_edge(self, node_a: str, node_b: str) -> MapEdge:
        """查找连接两个节点的边（无序）"""
        for edge in self._adj.get(node_a, []):
            if edge.other(node_a) == node_b:
                return edge
        raise KeyError(f"节点 {node_a} 与 {node_b} 之间没有直接连接")

    def has_edge(self, node_a: str, node_b: str) -> bool:
        try:
            self.get_edge(node_a, node_b)
            return True
        except KeyError:
            return False

    def reset_visit_status(self):
        """重置所有 mission 节点的打卡状态（用于新一轮测试）"""
        for node in self.nodes.values():
            if node.node_type == "mission":
                node.is_visited = False

    def get_mission_progress(self) -> Tuple[int, int]:
        """返回 (已打卡数, 总任务点数)"""
        visited = sum(1 for n in MISSION_NODES if self.nodes[n].is_visited)
        return visited, len(MISSION_NODES)

    def all_missions_completed(self) -> bool:
        """是否所有任务点均已打卡"""
        return all(self.nodes[n].is_visited for n in MISSION_NODES)

    def to_dict(self) -> Dict:
        """导出完整拓扑字典，便于 JSON 序列化"""
        return {
            "nodes": {name: node.to_dict() for name, node in self.nodes.items()},
            "edges": [edge.to_dict() for edge in self.edges],
        }


# 全局单例（模块级缓存）
_TOPOLOGY_INSTANCE: RaceTrackTopology = None


def get_topology() -> RaceTrackTopology:
    """获取全局赛道拓扑单例"""
    global _TOPOLOGY_INSTANCE
    if _TOPOLOGY_INSTANCE is None:
        _TOPOLOGY_INSTANCE = RaceTrackTopology()
    return _TOPOLOGY_INSTANCE
