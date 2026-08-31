"""
navigation/path_planner.py — 路径规划器

职责:
1. TSP 求解 → 缓存边序列 (edge_sequence)
2. 重规划判定（地图变更触发，正常完成不触发）
3. 边序列 → EdgeTask 转换
"""

from typing import List, Optional, Set
from dataclasses import dataclass, field

from ..contracts import EdgeTask, EdgeTaskStatus, PlanningInfeasibleError
from ..domain.line_graph import edge_heading, turn_angle_degs


@dataclass
class PathPlanResult:
    """路径规划结果"""
    node_sequence: List[str] = field(default_factory=list)   # 节点名序列
    edge_tasks: List[EdgeTask] = field(default_factory=list)  # 边任务序列
    total_distance_mm: float = 0.0
    needs_global_replan: bool = False  # 是否需要重跑 TSP
    returning_to_start: bool = False   # 是否处于回起点阶段（所有任务已完成）


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
               blocked_edges: Set[int] = None,
               extra_targets: List[str] = None,
               entry_heading: Optional[float] = None,
               required_edges: List[int] = None) -> PathPlanResult:
        """
        执行 TSP 全局规划。
        - 如果有未访问任务点（unvisited_nodes）或额外目标（extra_targets）：求解访问顺序 + 追加回 START
        - 如果全部完成：仅生成返回 START 的路径
        - 如果已在 START 且全部完成：返回空路径

        extra_targets 用于"涵洞扫荡"：额外必经节点（节点名）。

        entry_heading（Task 12 新增）：车头进入 current_node 时的朝向（世界 yaw_deg）。
        非 None 时，起点第一步的规划会禁止 180° 掉头并累加转向代价。

        required_edges（S-14 新增）：必须被「完整穿越」的边（edge_id 列表）。
        例如涵洞边——侦查涵洞要求车沿整条直道走一遍，而非仅仅到达某端口。
        这些边会作为"穿边目标"参与 TSP，并在后处理中保证被穿越（相邻出现
        node_a↔node_b），而不是只到 node_b 就折返。
        """
        blocked = blocked_edges or set()
        blocked_snap = frozenset(blocked)
        # S-14：targets 只含「打卡目标」（RFID center + extra_targets）。
        # required_edges（涵洞边）不进这里 —— 它们不是「打卡点」，而是「必须穿越的边」，
        # 由 _cover_required_edges 后处理保证穿越（避免把 port 误当归 TSP 目标，破坏
        # 端口级有向 TSP 的 center 打卡语义）。
        targets = list(unvisited_nodes)
        if extra_targets:
            for t in extra_targets:
                if t not in targets:
                    targets.append(t)
        req_edges = required_edges or []
        # 注：不再把 required_edge 的端口塞进 targets（见上）。

        returning = (len(targets) == 0)

        # 早退仅在「已在 START 且无剩余任务且无待穿越必边」时才发生。
        # 若还有 required_edges 未扫荡（如剩余的涵洞边），即使已在 START 也不能早退。
        if returning and current_node == "START" and not req_edges:
            # 已在起点且全部完成
            self._cached_sequence = ["START"]
            self._cached_tasks = []
            self._cursor = 0
            self._last_blocked_snapshot = blocked_snap
            return PathPlanResult(
                node_sequence=["START"],
                edge_tasks=[],
                total_distance_mm=0.0,
                needs_global_replan=False,
                returning_to_start=False,
            )

        if returning:
            # 任务全部完成，但不在 START → 用定向 Dijkstra 生成回程路径
            # （START 是基地节点，不能当打卡 TSP 目标）。
            # 注意：若在 START 但有 required_edges 待扫荡，这里 node_seq=[START]，
            # 后续 _cover_required_edges 会从 START 出发追加穿越段。
            node_seq = self._oracle.path_to_node(
                current_node, "START", blocked_edges=blocked,
                entry_heading=entry_heading,
            ) or [current_node]
        else:
            # 正常 TSP 规划：先访问所有任务点（含额外目标/穿边锚点）
            node_seq = self._oracle.query_shortest_path(
                current_node, targets, blocked_edges=blocked,
                entry_heading=entry_heading,
            )

        # S-14：穿边覆盖后处理 —— 保证每条 required_edge 被完整穿越
        # （在返回 START 之前插穿越段，避免「先回 START 再折返」）
        if req_edges:
            node_seq = self._cover_required_edges(node_seq, req_edges, blocked,
                                                  entry_heading)

        # 遍历/穿越完成后，追加回到 START 的路段（定向 Dijkstra，非 TSP）。
        # 方向 1：返程段也必须携带「进入 last_node 的朝向」（上一段末朝向），
        # 否则无向 Dijkstra 会自由反转方向，在拼接点产出 180° 掉头。
        last_node = node_seq[-1] if node_seq else current_node
        if last_node != "START":
            if len(node_seq) >= 2:
                arrival_heading = edge_heading(self._topo, node_seq[-2], node_seq[-1])
            else:
                arrival_heading = entry_heading
            return_path = self._oracle.path_to_node(
                last_node, "START", blocked_edges=blocked,
                entry_heading=arrival_heading,
            )
            # 合并：去掉 return_path 的第一个节点（与 last_node 重复）
            if return_path and len(return_path) > 1:
                node_seq = node_seq + return_path[1:]

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
            returning_to_start=returning,
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

        expected_yaw 取自「边自身的方向」（edge_heading，即有向边行驶朝向），
        而非逐段坐标差值反推（两者数值相等，但语义上朝向来自规划产物 DirectedStep，
        Task 12 落点）。
        """
        tasks = []
        for i in range(len(node_seq) - 1):
            a, b = node_seq[i], node_seq[i + 1]
            try:
                edge = self._topo.get_edge(a, b)
            except KeyError:
                continue

            expected_yaw = edge_heading(self._topo, a, b)

            tasks.append(EdgeTask(
                edge_id=edge.edge_id,
                from_node=a,
                to_node=b,
                expected_yaw=expected_yaw,
                distance_mm=edge.distance_mm,
                is_tunnel=edge.is_tunnel,
                speed_limit_ms=edge.speed_limit_ms,
            ))
        self._assert_no_uturn(tasks)
        return tasks

    def _assert_no_uturn(self, tasks: List[EdgeTask]) -> None:
        """
        兜底筛查（方向 2）：相邻两段边若构成「原地 180° 掉头」，直接抛错 fail fast。

        规划产物里出现 180° 掉头 = 规划层在无朝向（漏传 entry_heading）下生成了
        非法折返，是硬逻辑错误，绝不静默降级为 STOP/伪造转向（那会掩盖整车跑偏）。
        """
        for i in range(len(tasks) - 1):
            a, b = tasks[i], tasks[i + 1]
            diff = turn_angle_degs(a.expected_yaw, b.expected_yaw)
            if abs(diff) > 160.0:
                raise PlanningInfeasibleError(
                    f"非法 180° 掉头：{a.from_node}→{a.to_node}(yaw={a.expected_yaw:.1f}) "
                    f"→ {b.from_node}→{b.to_node}(yaw={b.expected_yaw:.1f}) diff={diff:.1f}"
                )

    # ================================================================
    # S-14：穿边覆盖后处理
    # ================================================================

    @staticmethod
    def _edge_crossed(seq: List[str], a: str, b: str) -> bool:
        """判断节点序列中是否相邻出现 (a,b) 或 (b,a)，即整条边被穿越。"""
        for i in range(len(seq) - 1):
            if (seq[i] == a and seq[i + 1] == b) or (seq[i] == b and seq[i + 1] == a):
                return True
        return False

    def _crossing_segment(self, edge_id: int, via_a: bool,
                          blocked: Set[int]) -> List[str]:
        """
        生成一条「穿越边」的节点段。

        穿越 edge(node_a, node_b) = [node_a, node_b] 或 [node_b, node_a]。
        via_a=True 表示从 node_a 进入、node_b 穿出；否则反向。
        返回完整穿越段（含两端端口）。
        """
        edge = self._topo.get_edge_by_id(edge_id)
        a, b = edge.node_a, edge.node_b
        return [a, b] if via_a else [b, a]

    def _cover_required_edges(self, node_seq: List[str],
                              req_edges: List[int],
                              blocked: Set[int],
                              entry_heading: Optional[float] = None) -> List[str]:
        """
        保证所有 required_edge 被完整穿越。

        策略：不改写 TSP 中段（避免产生「穿入又穿回」的 800mm 折返），
        而是在整个序列尾部追加以「穿越边」为目标的追加段：
        从当前尾节点出发，Dijkstra 到边的一端端口，穿到对端，继续下一条，
        最后回到 START。

        方向 1（188° 掉头 fail fast）：每次 `_dijkstra` 调用必须携带朝向；
        `entry_heading` 是「进入当前尾节点的朝向」，连续穿越段保持朝向连续，
        从规划层就杜绝 180° 首跳。
        """
        seq = list(node_seq)

        # 先去掉序列末尾「返回 START」的尾巴，之后统一追加
        # （本方法只在 returning 阶段调用，seq 尾部是 START 回程；但为通用，这里
        #   只处理仍未穿越的边，把穿越段构造在尾部回程之前。）
        uncrossed = []
        for eid in req_edges:
            try:
                edge = self._topo.get_edge_by_id(eid)
            except KeyError:
                continue
            a, b = edge.node_a, edge.node_b
            if not self._edge_crossed(seq, a, b):
                uncrossed.append((eid, a, b))

        if not uncrossed:
            return seq

        # 从当前尾节点出发，逐条穿越未覆盖边。
        # heading = 进入 cursor 时的朝向；初始是上一段 seq 的末段朝向，
        # 若无明确末段朝向则回退到调用方传入的 entry_heading。
        cursor = seq[-1]
        if len(seq) >= 2:
            heading = edge_heading(self._topo, seq[-2], seq[-1])
        else:
            heading = entry_heading
        appended = []
        for eid, a, b in uncrossed:
            # 规划 cursor → a（穿越 a→b）。a/b 是 port/普通节点，用 _dijkstra 直接算
            # （不是打卡 TSP，故不经过 query_shortest_path）。必须带朝向（方向 1）。
            d, path = self._oracle._dijkstra(cursor, a, blocked_edges=blocked,
                                             entry_heading=heading)
            if path is not None and len(path) > 1:
                appended.extend(path[1:])   # 到 a
                appended.append(b)          # 穿到 b
                cursor = b
                heading = edge_heading(self._topo, a, b)
            else:
                d, path = self._oracle._dijkstra(cursor, b, blocked_edges=blocked,
                                                 entry_heading=heading)
                if path is not None and len(path) > 1:
                    appended.extend(path[1:])   # 到 b
                    appended.append(a)          # 穿到 a
                    cursor = a
                    heading = edge_heading(self._topo, b, a)
                # 否则该边不可达，跳过（已通过连通性筛选，理论上不会发生）

        if appended:
            seq.extend(appended)

        return seq
