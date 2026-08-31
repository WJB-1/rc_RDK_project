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

from ..domain.topology import RaceTrackTopology, MapEdge, get_topology
from ..domain.config import EDGE_DEFAULTS
from ..domain.line_graph import edge_heading, turn_cost


class MapOracle:
    """
    地图真理查询器

    所有查询基于 RaceTrackTopology 中的静态拓扑数据，不修改任何节点状态。
    """

    def __init__(self, topology: RaceTrackTopology = None):
        self.topo = topology or get_topology()
        # 缓存所有节点对之间的最短路径（惰性计算）
        self._path_cache: Dict[tuple, Tuple[float, List[str]]] = {}
        self._dist_cache: Dict[tuple, float] = {}
        self._active_blocked: frozenset = frozenset()  # 当前查询的 blocked_edges，用于缓存 key
        self._active_entry_heading: Optional[float] = None  # 当前查询的进入朝向
        self._active_start_node: Optional[str] = None  # 当前查询的起点（用于决定是否施加朝向约束）

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

    def _internal_ports(self, center: str) -> List[str]:
        """返回到达该 center（方块中心）的内部半边端口列表。"""
        return [e.other(center) for e in self.topo.get_neighbors(center)
                if e.is_internal]

    def path_to_node(self, start_node: str, target_node: str,
                     blocked_edges: set = None,
                     entry_heading: Optional[float] = None) -> Optional[List[str]]:
        """
        定向最短路径：从 start_node 到 target_node 的单目标路径（非 TSP）。

        与 query_shortest_path（打卡 TSP）不同，这里 target 可以是任意图节点
        （如 START 基地），不做「进入端口→中心→离开端口」的打卡语义展开，
        直接用带朝向约束的 Dijkstra。返回完整节点序列或 None（不可达）。

        用于「返程回 START」等不需要打卡的场景，避免把 START 误当归 TSP 目标
        （START 无内部端口，会让端口级 TSP 无解而原地返回）。
        """
        blocked = frozenset(blocked_edges or set())
        d, path = self._dijkstra(start_node, target_node, blocked, entry_heading)
        return path

    def query_shortest_path(self, start_node: str,
                            unvisited_nodes: List[str],
                            blocked_edges: set = None,
                            entry_heading: Optional[float] = None) -> List[str]:
        """
        从当前点出发，遍历所有未打卡任务点的最优节点序列（端口级有向 TSP）。

        打卡一个 RFID 中心节点 = 从「进场端口」进入中心 → 打卡 → 从「出场端口」
        离开，且出场端口 ≠ 进场端口（禁止原路折返，即 180° 掉头）。
        Held-Karp DP 状态升级为 (mask, last_idx, last_exit_port)，两两距离按
        「上一个中心的出场端口 → 下一个中心的进场端口」的端口-端口最短路径计算，
        从而在搜索阶段就排除 corner 打卡后原路折返的非法序列。

        Args:
            start_node: 当前所在节点名称（可能是一个 center 或 port）
            unvisited_nodes: 仍未打卡的任务节点列表（RFID center）
            blocked_edges: 被障碍物封锁的 edge_id 集合
            entry_heading: 车头进入 start_node 时的朝向（世界 yaw_deg）。非 None 时
                起点第一步禁止 180° 掉头。

        Returns:
            List[str]: 包含 start_node 的最优遍历序列（含途径的中间端口节点）
        """
        if not unvisited_nodes:
            return [start_node]

        blocked = frozenset(blocked_edges or set())
        self._active_blocked = blocked
        self._active_entry_heading = entry_heading
        self._active_start_node = start_node

        targets = list(dict.fromkeys(unvisited_nodes))
        for t in targets:
            if t not in self.topo.nodes:
                raise KeyError(f"目标节点 {t} 不存在于拓扑图中")

        ports_of = {t: self._internal_ports(t) for t in targets}
        n = len(targets)
        INF = float('inf')

        # 端口-端口距离不能穿过「尚未打卡的 RFID center」。性能优化：先缓存
        # 「无 forbidden」的 (距离, 路径)，pd 只在该路径穿过 forbidden 时才带
        # forbidden 重算（这种重算也缓存）。
        _free_cache = {}
        def pd(a: str, b: str, entry: Optional[float] = None,
               forbidden: Optional[frozenset] = None) -> float:
            if a == b:
                return 0.0
            key = (a, b, entry)
            if key not in _free_cache:
                d, p = self._dijkstra(a, b, blocked, entry, None)
                _free_cache[key] = (d if p is not None else INF, p)
            d, p = _free_cache[key]
            if p is None:
                return INF
            if forbidden and any(x in forbidden for x in p[1:-1]):
                # 穿过了未打卡 center → 带 forbidden 重算
                d2, p2 = self._dijkstra(a, b, blocked, entry, forbidden)
                return d2 if p2 is not None else INF
            return d

        _pp_cache = {}
        def ppath(a: str, b: str, entry: Optional[float] = None,
                  forbidden: Optional[frozenset] = None) -> Optional[List[str]]:
            if a == b:
                return [a]
            key = (a, b, entry, forbidden)
            if key not in _pp_cache:
                _d, p = self._dijkstra(a, b, blocked, entry, forbidden)
                _pp_cache[key] = p
            return _pp_cache[key]

        # 预枚举每个 center 的合法 (arrive, depart) 端口对（depart != arrive，禁折返）
        pairs_of = {}
        for t in targets:
            pl = ports_of[t]
            pairs_of[t] = [(a, d) for a in pl for d in pl if a != d]
        # 记每个 center 的坐标/朝向，便于重建时算 depart 朝向
        from ..domain.line_graph import edge_heading as _eh

        # ---- 有向 TSP DP：状态 (mask, last_idx, arrive_port, depart_port) ----
        # 状态自含 arrive/depart，回溯无歧义。规模 2^n × Σ(组合数) ≈ 4k×48，可控。
        dp = {}  # state -> (cost, prev_state)
        start_type = self.topo.nodes[start_node].node_type if start_node in self.topo.nodes else ''

        # forbidden = 尚未打卡的 targets（含即将打卡的 nxt，因为去它的 arrive 端口
        # 不能穿过它本身）。已打卡的 center 可穿越（否则障碍物封锁后无路绕行）。
        def forbidden_upto(mask: int) -> frozenset:
            # 未打卡（bit 未置位）的 targets —— 含即将打卡的那一个
            return frozenset(targets[j] for j in range(n) if not (mask >> j & 1))

        # 初始化：从 start_node 出发，打卡第一个 target。
        # 此时尚未打卡任何 target，故 forbidden = 全部 targets（含第一个 i），
        # 去第一个 arrive 端口的路上不得穿过任何 center（包括 i 本身）。
        for i in range(n):
            t = targets[i]
            for (arr, dep) in pairs_of[t]:
                forb = forbidden_upto(0)
                if start_type in ('port', 'base'):
                    road = pd(start_node, arr, None, forb)
                else:
                    # start 是 center：从其中心出发（entry_heading 约束），算到 arr
                    road = self._dijkstra(start_node, arr, blocked, entry_heading, forb)[0]
                if road >= INF:
                    continue
                # arrive(100) + depart(100) + 道路
                cost = road + 200.0
                state = (1 << i, i, arr, dep)
                if cost < dp.get(state, (INF, None))[0]:
                    dp[state] = (cost, None)

        # 状态转移
        for mask in range(1, 1 << n):
            for last in range(n):
                if not (mask & (1 << last)):
                    continue
                t_last = targets[last]
                for (arr_l, dep_l) in pairs_of[t_last]:
                    state = (mask, last, arr_l, dep_l)
                    if state not in dp:
                        continue
                    cur_cost, _ = dp[state]
                    for nxt in range(n):
                        if mask & (1 << nxt):
                            continue
                        t_nxt = targets[nxt]
                        for (arr_n, dep_n) in pairs_of[t_nxt]:
                            # 段间距离带入射朝向：车驶离 t_last 朝 dep_l 的方向。
                            # forbidden = mask 尚未打卡的 targets（含 t_nxt，去其 arrive
                            # 端口不能穿 t_nxt 本身）。已打卡的 t_last 等可穿越。
                            forb = forbidden_upto(mask)
                            road = pd(dep_l, arr_n, _eh(self.topo, t_last, dep_l), forb)
                            if road >= INF:
                                continue
                            new_state = (mask | (1 << nxt), nxt, arr_n, dep_n)
                            new_cost = cur_cost + road + 200.0
                            if new_cost < dp.get(new_state, (INF, None))[0]:
                                dp[new_state] = (new_cost, state)

        full_mask = (1 << n) - 1
        best_state = None
        best_cost = INF
        for last in range(n):
            for (arr, dep) in pairs_of[targets[last]]:
                st = (full_mask, last, arr, dep)
                if st in dp and dp[st][0] < best_cost:
                    best_cost = dp[st][0]
                    best_state = st
        if best_state is None:
            return [start_node]

        # 回溯：还原 (center, arrive_port, depart_port) 序列（正序）
        steps = []
        st = best_state
        while st is not None:
            cost, prev = dp[st]
            mask, last, arr, dep = st
            steps.append((targets[last], arr, dep))
            st = prev
        steps.reverse()

        # 重建完整节点路径
        full = self._rebuild_directed_path(start_node, steps, ppath,
                                           frozenset(targets))
        return full

    # ============================================================
    # 内部辅助
    # ============================================================

    def _rebuild_directed_path(self, start_node: str,
                               steps: List[Tuple[str, str, str]],
                               ppath,
                               all_targets: frozenset) -> List[str]:
        """
        根据有向 TSP 的端口级 step 序列重建完整节点路径。

        steps: [(center, arrive_port, depart_port), ...]（正序）
        路径 = start → arrive0 → center0 → depart0 → arrive1 → center1 → depart1 → ...

        关键：段间必须携带「入射朝向」（车在 depart_port 的朝向）与「未打卡点集合」，
        Dijkstra 第一步不会折返、也不穿过尚未打卡的 center。
        """
        from ..domain.line_graph import edge_heading as _eh
        full: List[str] = [start_node]

        prev_heading = self._active_entry_heading  # 起点入射朝向（可能 None）
        forbidden = set(all_targets)  # 尚未打卡的 center
        for idx, (center, arrive, depart) in enumerate(steps):
            # 去 arrive 端口的路不能穿过 center 本身（它是打卡终点）——
            # 故 forbid 里保留 center，不含「已打卡」的（forbidden 已递减）也无妨。
            forbid_now = frozenset(forbidden)
            if idx == 0:
                # 起点 → arrive0（带起点入射朝向 + 未打卡点）
                seg = ppath(start_node, arrive, prev_heading, forbid_now)
                if seg is None:
                    seg = [start_node, arrive]
                full.extend(seg[1:])
            else:
                # 上一段 depart → 本段 arrive（带上一段末朝向 + 未打卡点）
                seg = ppath(steps[idx - 1][2], arrive, prev_heading, forbid_now)
                if seg is None:
                    seg = [steps[idx - 1][2], arrive]
                full.extend(seg[1:])
            # arrive → center → depart
            full.append(center)
            full.append(depart)
            # 更新：center 已打卡，从未打卡集合移除；朝向 = 驶离 center 朝 depart
            forbidden.discard(center)
            prev_heading = _eh(self.topo, center, depart)

        return full

    @staticmethod
    def _cache_key(a: str, b: str, blocked: frozenset,
                   entry_heading: Optional[float] = None) -> tuple:
        """生成包含 blocked_edges 与进入朝向的缓存 key，确保不同封锁/朝向状态不复用缓存路径。"""
        return (a, b, blocked, entry_heading)

    def _precompute_paths(self, node_names: List[str], blocked_edges: set = None,
                          entry_heading: Optional[float] = None):
        """
        对给定的节点列表，预计算两两之间的最短路径。

        entry_heading 仅对「从起点 node_names[0] 出发」的边施加转向约束
        （禁止掉头 + 累加转向代价）；起点与其他节点之间、以及目标节点之间，
        均保持无向距离（node_names[0] 之后的元素是任务节点，它们之间的转向
        约束属于有向 TSP，见 path_planner 的上层拼接，不在本层展开）。
        """
        blocked = frozenset(blocked_edges or set())
        start_node = node_names[0]
        for i, a in enumerate(node_names):
            for b in node_names[i + 1:]:
                # 仅当 pair 的一端是起点，且提供 entry_heading 时，才对起点侧施加朝向约束
                heading = entry_heading if a == start_node else None
                key = self._cache_key(a, b, blocked, heading)
                if key not in self._dist_cache:
                    dist, path = self._dijkstra(a, b, blocked, heading)
                    if path is not None:
                        self._dist_cache[key] = dist
                        self._dist_cache[self._cache_key(b, a, blocked)] = dist
                        self._path_cache[key] = (dist, path)
                        self._path_cache[self._cache_key(b, a, blocked)] = (dist, list(reversed(path)))

    def _get_dist(self, a: str, b: str) -> Optional[float]:
        """获取缓存的两节点最短距离（使用当前 blocked 状态作为 key 的一部分）"""
        if a == b:
            return 0.0
        return self._dist_cache.get(self._cache_key(a, b, self._active_blocked,
                                                     self._heading_for(a)))

    def _get_path(self, a: str, b: str) -> Optional[List[str]]:
        """获取缓存的两节点最短路径（包含端点，使用当前 blocked 状态作为 key 的一部分）"""
        if a == b:
            return [a]
        entry = self._path_cache.get(self._cache_key(a, b, self._active_blocked,
                                                      self._heading_for(a)))
        return entry[1] if entry else None

    def _heading_for(self, a: str) -> Optional[float]:
        """返回该节点作为「起点」时应施加的进入朝向（仅起点节点为 entry_heading，其余为 None）。"""
        if self._active_entry_heading is not None and a == self._active_start_node:
            return self._active_entry_heading
        return None

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
                  blocked_edges: set = None,
                  entry_heading: Optional[float] = None,
                  forbidden_mid: Optional[set] = None) -> Tuple[float, Optional[List[str]]]:
        """
        Dijkstra 最短路径算法（带朝向 / 状态空间版）。

        - blocked_edges：封锁边不再通行。
        - entry_heading：车进入 start 时的朝向。非 None 时，start 第一步禁止
          180° 掉头并累加转向代价。
        - 每条边在「中间节点」也按转角累加转向代价（直行 0，±90° 300mm），
          使两两最短距离变成真正带朝向的距离；掉头（180°）转移被排除。

        状态空间用 (当前节点, 前驱节点) 表达，前驱用于计算转角。为避免在
        port→center→port 这类「路口方块中心折返」上的伪掉头误判，只在
        「前驱、当前、后继三点成直线反折(180°)」时排除转移。

        隧道不再 *0.6 缩水，与普通边同代价（Task 12 S-13 整改）。

        Returns:
            (distance_mm, path_list) 如果可达；否则 (inf, None)
        """
        if start == goal:
            return 0.0, [start]

        blocked = blocked_edges or set()
        # 状态空间 Dijkstra：状态 = (node, prev)，dist 记录到该状态的最小代价
        dist: Dict[Tuple[str, Optional[str]], float] = {}
        prev: Dict[Tuple[str, Optional[str]], Optional[Tuple[str, Optional[str]]]] = {}
        start_state = (start, None)
        dist[start_state] = 0.0

        heap = [(0.0, start, None)]
        reached = None  # 首次弹出 goal 时记录的状态

        while heap:
            d, u, p = heapq.heappop(heap)
            state = (u, p)
            if d > dist.get(state, math.inf):
                continue
            if u == goal:
                reached = state
                break

            for edge in self.topo.get_neighbors(u):
                if edge.edge_id in blocked:
                    continue
                v = edge.other(u)
                # 禁止作为「中间节点」通过（除非它是终点）。用于端口-端口距离，
                # 避免最短路径穿过未打卡的 RFID center（会用打卡点提前路过 + 折返）。
                if forbidden_mid and v in forbidden_mid and v != goal:
                    continue

                # 转角计算：进入朝向 = 来自 p→u 的方向（起点用 entry_heading）
                if p is None:
                    in_heading = entry_heading
                else:
                    in_heading = edge_heading(self.topo, p, u)
                out_heading = edge_heading(self.topo, u, v)

                if in_heading is None:
                    tc = 0.0
                else:
                    tc = turn_cost(in_heading, out_heading)
                    if math.isinf(tc):
                        continue  # 180° 掉头，禁止

                w = edge.distance_mm + tc
                nd = d + w
                nxt_state = (v, u)
                if nd < dist.get(nxt_state, math.inf):
                    dist[nxt_state] = nd
                    prev[nxt_state] = state
                    heapq.heappush(heap, (nd, v, u))

        if reached is None:
            return math.inf, None

        # 回溯路径（沿 prev 链）
        path = []
        state = reached
        while state is not None:
            path.append(state[0])
            state = prev.get(state)
        path.reverse()
        return dist[reached], path

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
