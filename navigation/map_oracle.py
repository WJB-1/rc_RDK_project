"""
navigation/map_oracle.py - 地图真理查询 API (Map Oracle)

基于《物理地图建模与Agent接口规范.md》第3.3节实现。
提供纯静态/被动状态查询接口，为 Dijkstra / TSP 算法提供支持。

核心功能：
- query_shortest_path: 从当前点出发，遍历所有未打卡任务点的最优节点序列
- get_edge_properties: 查询两相连节点之间的物理属性
"""

import math
import heapq
from typing import Dict, List, Tuple, Optional

from .map_topology import RaceTrackTopology, MapEdge, get_topology
from .map_config import EDGE_DEFAULTS


class MapOracle:
    """
    地图真理查询器

    所有查询基于 RaceTrackTopology 中的静态拓扑数据，不修改任何节点状态。
    """

    def __init__(self, topology: RaceTrackTopology = None):
        self.topo = topology or get_topology()
        # 缓存所有节点对之间的最短路径（惰性计算）
        self._path_cache: Dict[Tuple[str, str], Tuple[float, List[str]]] = {}
        self._dist_cache: Dict[Tuple[str, str], float] = {}

    # ============================================================
    # 公共 API
    # ============================================================

    def get_edge_properties(self, node_a: str, node_b: str) -> Dict:
        """
        查询两个相连节点之间的物理属性。

        返回示例:
            {
                "distance_mm": 800,
                "is_tunnel": true,
                "has_culvert": false,
                "speed_limit_ms": 0.30
            }
        """
        try:
            edge = self.topo.get_edge(node_a, node_b)
            return edge.to_dict()
        except KeyError as exc:
            raise KeyError(f"节点 {node_a} 与 {node_b} 之间没有直接边") from exc

    def query_shortest_path(self, start_node: str,
                            unvisited_nodes: List[str],
                            blocked_edges: set = None) -> List[str]:
        """
        从当前点出发，遍历所有未打卡任务点的最优节点序列。

        算法：
        1. 预计算所有节点对的 Dijkstra 最短路径；
        2. 对未打卡点集合使用 Held-Karp (状态压缩 DP) 求解近似最优遍历顺序。

        Args:
            start_node: 当前所在节点名称
            unvisited_nodes: 仍未打卡的任务节点列表
            blocked_edges: 被障碍物封锁的 edge_id 集合（这些边不可通行）

        Returns:
            List[str]: 包含 start_node 的最优遍历序列，例如 ["N2", "T1_L", "T1_R", "N11"]
        """
        if not unvisited_nodes:
            return [start_node]

        blocked_edges = blocked_edges or set()

        # 去重并验证
        targets = list(dict.fromkeys(unvisited_nodes))
        for t in targets:
            if t not in self.topo.nodes:
                raise KeyError(f"目标节点 {t} 不存在于拓扑图中")

        # 1. 计算所有相关节点对之间的最短距离与路径（传入 blocked_edges）
        relevant_nodes = [start_node] + targets
        self._precompute_paths(relevant_nodes, blocked_edges)

        # 2. Held-Karp DP
        n = len(targets)
        if n == 0:
            return [start_node]

        # 状态: (mask, last_idx) -> (min_cost, prev_state)
        # mask 是 targets 的位掩码，last_idx 是最后访问的 target 索引
        INF = float('inf')
        dp: Dict[Tuple[int, int], Tuple[float, Optional[Tuple[int, int]]]] = {}

        # 初始化：从 start_node 到每个 target
        for i in range(n):
            dist = self._get_dist(start_node, targets[i])
            if dist is None:
                continue
            mask = 1 << i
            dp[(mask, i)] = (dist, None)

        # 状态转移
        for mask in range(1, 1 << n):
            for last in range(n):
                if not (mask & (1 << last)):
                    continue
                state = (mask, last)
                if state not in dp:
                    continue
                cur_cost, _ = dp[state]
                for nxt in range(n):
                    if mask & (1 << nxt):
                        continue
                    new_mask = mask | (1 << nxt)
                    d = self._get_dist(targets[last], targets[nxt])
                    if d is None:
                        continue
                    new_cost = cur_cost + d
                    new_state = (new_mask, nxt)
                    if new_state not in dp or new_cost < dp[new_state][0]:
                        dp[new_state] = (new_cost, state)

        # 寻找最优终点
        full_mask = (1 << n) - 1
        best_cost = INF
        best_state = None
        for last in range(n):
            state = (full_mask, last)
            if state in dp and dp[state][0] < best_cost:
                best_cost = dp[state][0]
                best_state = state

        if best_state is None:
            # 不可达（理论上不会发生）
            return [start_node]

        # 回溯得到 targets 的访问顺序
        order_rev = []
        state = best_state
        while state is not None:
            _, prev = dp[state]
            mask, last = state
            order_rev.append(targets[last])
            state = prev

        order = list(reversed(order_rev))

        # 展开为实际路径节点序列（包含途径的中间节点）
        full_path = self._expand_path([start_node] + order)
        return full_path

    # ============================================================
    # 内部辅助
    # ============================================================

    def _precompute_paths(self, node_names: List[str], blocked_edges: set = None):
        """对给定的节点列表，预计算两两之间的最短路径"""
        blocked = blocked_edges or set()
        for i, a in enumerate(node_names):
            for b in node_names[i + 1:]:
                if (a, b) not in self._dist_cache:
                    dist, path = self._dijkstra(a, b, blocked)
                    if path is not None:
                        self._dist_cache[(a, b)] = dist
                        self._dist_cache[(b, a)] = dist
                        self._path_cache[(a, b)] = (dist, path)
                        self._path_cache[(b, a)] = (dist, list(reversed(path)))

    def _get_dist(self, a: str, b: str) -> Optional[float]:
        """获取缓存的两节点最短距离"""
        if a == b:
            return 0.0
        return self._dist_cache.get((a, b))

    def _get_path(self, a: str, b: str) -> Optional[List[str]]:
        """获取缓存的两节点最短路径（包含端点）"""
        if a == b:
            return [a]
        entry = self._path_cache.get((a, b))
        return entry[1] if entry else None

    def _expand_path(self, node_sequence: List[str]) -> List[str]:
        """
        将节点序列展开为完整路径（在连续节点之间插入 Dijkstra 最短路径的中间节点）。
        注意去重：如果上一段的终点是下一段的起点，避免重复。
        """
        if not node_sequence:
            return []
        full = [node_sequence[0]]
        for i in range(len(node_sequence) - 1):
            a, b = node_sequence[i], node_sequence[i + 1]
            sub = self._get_path(a, b)
            if sub is None:
                sub = [a, b]
            # 跳过 sub 的第一个元素（与 full 最后一个重复）
            full.extend(sub[1:])
        return full

    def _dijkstra(self, start: str, goal: str,
                  blocked_edges: set = None) -> Tuple[float, Optional[List[str]]]:
        """
        Dijkstra 最短路径算法。

        支持 blocked_edges（不可通行的边）和隧道加分权重。

        Returns:
            (distance_mm, path_list) 如果可达；否则 (inf, None)
        """
        if start == goal:
            return 0.0, [start]

        blocked = blocked_edges or set()
        dist: Dict[str, float] = {name: math.inf for name in self.topo.nodes}
        prev: Dict[str, Optional[str]] = {name: None for name in self.topo.nodes}
        dist[start] = 0.0

        heap = [(0.0, start)]
        visited: set = set()

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)
            if u == goal:
                break

            for edge in self.topo.get_neighbors(u):
                # 跳过被封锁的边
                if edge.edge_id in blocked:
                    continue
                v = edge.other(u)
                w = edge.distance_mm
                # 隧道加分：降低权重，TSP 优先选择
                if edge.is_tunnel:
                    w *= 0.6
                if d + w < dist[v]:
                    dist[v] = d + w
                    prev[v] = u
                    heapq.heappush(heap, (dist[v], v))

        if dist[goal] == math.inf:
            return math.inf, None

        # 回溯路径
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = prev[node]
        path.reverse()
        return dist[goal], path

    def get_path_details(self, node_sequence: List[str]) -> List[Dict]:
        """
        给定节点序列，返回每一段的详细边属性（用于调试/可视化）。

        Returns:
            [
                {"from": "N1", "to": "N2", "distance_mm": 800, "is_tunnel": True, ...},
                ...
            ]
        """
        details = []
        for i in range(len(node_sequence) - 1):
            a, b = node_sequence[i], node_sequence[i + 1]
            try:
                props = self.get_edge_properties(a, b)
                details.append(props)
            except KeyError:
                details.append({
                    "from": a,
                    "to": b,
                    "error": "无直接连接，属于多跳路径",
                })
        return details
