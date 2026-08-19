"""
navigation/map_topology.py - 赛道全局拓扑与节点物理坐标定义

基于《物理地图建模与Agent接口规范.md》第1、2节，采用"端口模型"实现。

端口模型（积木式）：
  JunctionCenter（方块中心）→ JunctionBoundaryPort（端口）→ RoadSegment（净直道）

两类边：
  1. 方块内部半边（center-port，is_internal=True）：方块中心连到自己的每个端口，
     距离恒为 BLOCK_HALF_MM = 100mm，是"转弯/连接"边，让车能从方块中心走到任一端口的直道。
  2. 直道/桥（port-port，is_internal=False）：连接相邻方块的端口，距离 = 净直道长
     (800/300/200mm)，其中隧道段 is_tunnel=True。

图节点 = 方块中心（含 START base 锚点） + 端口节点。

拓扑结构:

       START (0,0)
         | 200mm (启动桥)
         ▼
    J_START (0,300)  ← T型路口
    ───┬────────┬───
   N1(-500)   N12(500)   ← 顶部拐角
      │          │
    T1_L ==[隧道]== T1_R   ← 内环第一行
      │          │
    T2_L ==[隧道]== T2_R
      │          │
    T3_L ==[隧道]== T3_R
      │          │
     N6 ── N7 内环底部
    N2─N3─N4─N5 (外环左列)   N11─N10─N9─N8 (外环右列)
      └──────────────┴──────────────┘
"""

import math
from typing import Dict, List, Tuple

from .config import (
    NODE_COORDS,
    NODE_TYPES,
    NODE_HAS_RFID,
    MISSION_NODES,
    EDGE_DEFAULTS,
    TUNNEL_EDGE_DEFAULTS,
    STRAIGHT_SEGMENT_LENGTH_MM,
    START_DEPTH_MM,
    BLOCK_HALF_MM,
)


class MapNode:
    """地图节点数据结构"""

    def __init__(self, name: str, x_mm: float, y_mm: float, node_type: str, has_rfid: bool):
        self.name = name
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.node_type = node_type       # "base" | "junction-T" | "junction-cross" | "corner" | "port"
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

    静态属性（编译期确定）:
      node_a/b, distance_mm, is_tunnel, at_speed_limit_ms,
      is_internal（是否方块内部半边 center-port）

    动态属性（运行时更新）: is_blocked, has_culvert, is_reconned, visit_count, edge_id

    has_culvert 语义（2026-08-14 收窄）：仅表达「发现过涵洞」（首次 on_culvert_detected
    或路口侧视发现时置 True）。
    is_reconned 语义：表达「侦查完成」（真正驶过/进入 CULVERT_RECON 完成后置 True）。
    两者分离，允许「发现过但未侦查」的中间态。

    is_internal 字段区分两类边：
      - True:  方块内部半边（center 到自己的端口，distance=100mm），用于转弯
      - False: 直道/桥（port-port），其中 is_tunnel=True 表示为隧道段；
               仅有非隧道直道 (is_internal=False, is_tunnel=False) 才可放涵洞/障碍物
    """

    _next_edge_id = 0

    def __init__(self, node_a: str, node_b: str, distance_mm: float,
                 is_tunnel: bool = False, has_culvert: bool = False,
                 speed_limit_ms: float = None, is_internal: bool = False):
        self.edge_id = MapEdge._next_edge_id
        MapEdge._next_edge_id += 1
        self.node_a = node_a
        self.node_b = node_b
        self.distance_mm = distance_mm
        self.is_tunnel = is_tunnel
        self.has_culvert = has_culvert
        self.is_internal = is_internal
        self.speed_limit_ms = speed_limit_ms or (
            TUNNEL_EDGE_DEFAULTS["speed_limit_ms"] if is_tunnel else EDGE_DEFAULTS["speed_limit_ms"]
        )
        # 运行时动态属性
        self.is_blocked: bool = False
        self.is_reconned: bool = False  # 侦查完成（真正驶过/进入 CULVERT_RECON 完成）
        self.visit_count: int = 0

    def __repr__(self) -> str:
        return (
            f"MapEdge({self.node_a} <-> {self.node_b}, d={self.distance_mm}mm, "
            f"tunnel={self.is_tunnel}, internal={self.is_internal}, "
            f"blocked={self.is_blocked}, culvert={self.has_culvert}, "
            f"reconned={self.is_reconned}, visits={self.visit_count})"
        )

    def to_dict(self) -> Dict:
        return {
            "edge_id": self.edge_id,
            "node_a": self.node_a,
            "node_b": self.node_b,
            "distance_mm": self.distance_mm,
            "is_tunnel": self.is_tunnel,
            "has_culvert": self.has_culvert,
            "is_reconned": self.is_reconned,
            "is_internal": self.is_internal,
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

    节点命名规范（端口模型）：
    - START: 出发区 base 锚点（非方块）
    - J_START: START 与赛道交点 (T型路口方块中心)
    - N1~N12: 必到任务点（方块中心，含 RFID）
    - T1_L/T1_R, T2_L/T2_R, T3_L/T3_R: 三行隧道的十字路口方块中心
    - {方块名}.P_{N/E/S/W}: 方块四方向端口

    边命名规范：
    - 方块内部半边: center <-> 端口 (is_internal=True)
    - 直道/桥: 端口 <-> 端口 (is_internal=False)
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
    # 构建边（严格按照端口模型拓扑）
    # ------------------------------------------
    def _build_edges(self):
        """
        端口模型拓扑连接关系（无向边）。

        生成顺序（固定，保证边 id 可复现）：
          1. 方块内部半边（center-port，is_internal=True）
             - 启动桥 START -> J_START.P_N (200mm)
          2. 直道/桥（port-port，is_internal=False）

        拓扑示意：
          START --[启动桥]--> J_START.P_N
          J_START --┬ 770port ┬-- N12
          N1 --- T1_L==[隧道]==T1_R --- N11
          N2 --- T2_L==[隧道]==T2_R --- N10
          N3 --- T3_L==[隧道]==T3_R --- N9
          N4 --- N6 ====== N7 --- N8
                (内环底) (外环底)
        """

        self._build_internal_edges()
        self._build_straight_edges()

    # ------------------------------------------
    # 边类型 1：方块内部半边（center-port，100mm）
    # ------------------------------------------
    def _build_internal_edges(self):
        """
        程序化生成每个方块 center 到自己的所有端口的半边。
        跳过 base 与 port 节点，仅对方块中心（junction-*/corner）生成。
        """
        for name, node in self.nodes.items():
            ntype = node.node_type
            # 仅对"方块中心"节点生成（非 base、非 port）
            if ntype in ("base", "port"):
                continue
            # 收集该方块的所有 `${name}.P_方向` 端口
            for port_dir in ("N", "S", "E", "W"):
                port_name = f"{name}.P_{port_dir}"
                if port_name in self.nodes:
                    self._add_edge(name, port_name, BLOCK_HALF_MM,
                                   False, False, is_internal=True)

        # 启动桥: START -> J_START.P_N（200mm，含一个方块半长的贯通段）
        self._add_edge("START", "J_START.P_N", START_DEPTH_MM,
                       False, False, is_internal=False)

    # ------------------------------------------
    # 边类型 2：直道/桥（port-port）
    # ------------------------------------------
    def _build_straight_edges(self):
        """直道/桥边（端口-端口）。"""
        LO = STRAIGHT_SEGMENT_LENGTH_MM  # 800mm

        edge_specs = [
            # 顶部分叉（J_START 到 N1/N12）
            ("J_START.P_W", "N1.P_E", 300, False, False),
            ("J_START.P_E", "N12.P_W", 300, False, False),
            # 内环左列
            ("N1.P_S", "T1_L.P_N", 800, False, False),
            ("T1_L.P_S", "T2_L.P_N", 800, False, False),
            ("T2_L.P_S", "T3_L.P_N", 800, False, False),
            ("T3_L.P_S", "N6.P_N", 800, False, False),
            # 内环右列
            ("N12.P_S", "T1_R.P_N", 800, False, False),
            ("T1_R.P_S", "T2_R.P_N", 800, False, False),
            ("T2_R.P_S", "T3_R.P_N", 800, False, False),
            ("T3_R.P_S", "N7.P_N", 800, False, False),
            # 隧道
            ("T1_L.P_E", "T1_R.P_W", 800, True, False),
            ("T2_L.P_E", "T2_R.P_W", 800, True, False),
            ("T3_L.P_E", "T3_R.P_W", 800, True, False),
            # 外环左列
            ("N2.P_S", "N3.P_N", 800, False, False),
            ("N3.P_S", "N4.P_N", 800, False, False),
            ("N4.P_S", "N5.P_N", 800, False, False),
            # 外环右列
            ("N11.P_S", "N10.P_N", 800, False, False),
            ("N10.P_S", "N9.P_N", 800, False, False),
            ("N9.P_S", "N8.P_N", 800, False, False),
            # 内外横向连接（左）
            ("T1_L.P_W", "N2.P_E", 800, False, False),
            ("T2_L.P_W", "N3.P_E", 800, False, False),
            ("T3_L.P_W", "N4.P_E", 800, False, False),
            # 内外横向连接（右）
            ("T1_R.P_E", "N11.P_W", 800, False, False),
            ("T2_R.P_E", "N10.P_W", 800, False, False),
            ("T3_R.P_E", "N9.P_W", 800, False, False),
            # 底部
            ("N5.P_E", "N6.P_W", 800, False, False),
            ("N6.P_E", "N7.P_W", 800, True, False),
            ("N7.P_E", "N8.P_W", 800, False, False),
        ]

        for a, b, dist, is_tunnel, has_culvert in edge_specs:
            self._add_edge(a, b, dist, is_tunnel, has_culvert, is_internal=False)

    # ------------------------------------------
    # 添加边
    # ------------------------------------------
    def _add_edge(self, a: str, b: str, dist: float, is_tunnel: bool = False,
                  has_culvert: bool = False, is_internal: bool = False):
        edge = MapEdge(a, b, dist, is_tunnel, has_culvert, is_internal=is_internal)
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

    def get_edge_by_id(self, edge_id: int) -> MapEdge:
        """
        按 edge_id 查找边（用于场景生成等，edge_id 是全局递增类变量，
        不一定等于 edges 数组下标，不能直接 topo.edges[eid]）。
        """
        for edge in self.edges:
            if edge.edge_id == edge_id:
                return edge
        raise KeyError(f"边 {edge_id} 不存在")

    def has_edge(self, node_a: str, node_b: str) -> bool:
        try:
            self.get_edge(node_a, node_b)
            return True
        except KeyError:
            return False

    def reset_visit_status(self):
        """重置所有 mission 节点的打卡状态（用于新一轮测试）"""
        for node in self.nodes.values():
            if node.name in MISSION_NODES:
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
