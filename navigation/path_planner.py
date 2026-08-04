"""
navigation/path_planner.py — 路径规划器

职责:
1. TSP 求解 → 缓存边序列 (edge_sequence)
2. 重规划判定（地图变更触发，正常完成不触发）
3. 边序列 → EdgeTask 转换
"""

from typing import List, Optional, Set
from dataclasses import dataclass, field

from .contracts import EdgeTask, EdgeTaskStatus


@dataclass
class PathPlanResult:
    """路径规划结果"""
    node_sequence: List[str] = field(default_factory=list)   # 节点名序列
    edge_tasks: List[EdgeTask] = field(default_factory=list)  # 边任务序列
    total_distance_mm: float = 0.0
    needs_global_replan: bool = False  # 是否需要重跑 TSP


class PathPlanner:
    """
    路径规划器 — 管理全局路径缓存与重规划判定

    使用方式:
        planner = PathPlanner(oracle, topo)
        planner.replan(current_node, unvisited, blocked_edges)  # 首次/地图变更
        task = planner.next_task()                                # 取下一条边

    缓存策略:
        - 首次启动: 强制 TSP
        - 地图变更 (blocked_edges 变化): 强制 TSP
        - 正常完成一条边: 只推进缓存指针，不重跑 TSP
        - RFID 打卡 (visited 变化): 可能触发 TSP（如果影响最优解）
    """

    def __init__(self, oracle, topo):
        self._oracle = oracle
        self._topo = topo
        self._cached_sequence: List[str] = []      # 节点名序列
        self._cached_tasks: List[EdgeTask] = []     # 边任务序列
        self._cursor: int = 0                       # 当前执行到第几条边
        self._last_blocked_snapshot: frozenset = frozenset()

    # ================================================================
    # 公开接口
    # ================================================================

    def replan(self, current_node: str, unvisited_nodes: List[str],
               blocked_edges: Set[int] = None) -> PathPlanResult:
        """
        执行 TSP 全局规划。
        仅在以下情况调用:
        - 首次启动
        - 地图发生预料之外变更（blocked_edges 变化）
        """
        blocked = blocked_edges or set()
        blocked_snap = frozenset(blocked)

        node_seq = self._oracle.query_shortest_path(
            current_node, unvisited_nodes, blocked_edges=blocked
        )
        tasks = self._node_seq_to_tasks(node_seq)

        self._cached_sequence = node_seq
        self._cached_tasks = tasks
        self._cursor = 0
        self._last_blocked_snapshot = blocked_snap

        total_dist = sum(t.distance_mm for t in tasks)
        return PathPlanResult(
            node_sequence=node_seq,
            edge_tasks=tasks,
            total_distance_mm=total_dist,
            needs_global_replan=False,
        )

    def next_task(self) -> Optional[EdgeTask]:
        """从缓存中取下一任务"""
        if self._cursor < len(self._cached_tasks):
            task = self._cached_tasks[self._cursor]
            self._cursor += 1
            return task
        return None

    def peek_task(self) -> Optional[EdgeTask]:
        """查看当前任务但不推进指针"""
        if self._cursor < len(self._cached_tasks):
            return self._cached_tasks[self._cursor]
        return None

    def should_replan(self, blocked_edges: Set[int] = None,
                      visited_nodes: Set[str] = None) -> bool:
        """
        判定是否需要重规划。

        触发条件:
        1. blocked_edges 集合自上次规划后发生变化
        """
        blocked = blocked_edges or set()
        if frozenset(blocked) != self._last_blocked_snapshot:
            return True
        return False

    def has_next(self) -> bool:
        return self._cursor < len(self._cached_tasks)

    def get_cached_sequence(self) -> List[str]:
        return list(self._cached_sequence)

    # ================================================================
    # 内部
    # ================================================================

    def _node_seq_to_tasks(self, node_seq: List[str]) -> List[EdgeTask]:
        """
        将节点序列转换为边任务序列。

        例如 ["N2", "T1_L", "T1_R", "N11"]
          → [EdgeTask(N2→T1_L), EdgeTask(T1_L→T1_R), EdgeTask(T1_R→N11)]
        """
        import math
        tasks = []
        for i in range(len(node_seq) - 1):
            a, b = node_seq[i], node_seq[i + 1]
            try:
                edge = self._topo.get_edge(a, b)
            except KeyError:
                continue

            dx = self._topo.nodes[b].x_mm - self._topo.nodes[a].x_mm
            dy = self._topo.nodes[b].y_mm - self._topo.nodes[a].y_mm
            expected_yaw = math.degrees(math.atan2(dx, dy))

            tasks.append(EdgeTask(
                edge_id=edge.edge_id,
                from_node=a,
                to_node=b,
                expected_yaw=expected_yaw,
                distance_mm=edge.distance_mm,
                is_tunnel=edge.is_tunnel,
                speed_limit_ms=edge.speed_limit_ms,
            ))
        return tasks
