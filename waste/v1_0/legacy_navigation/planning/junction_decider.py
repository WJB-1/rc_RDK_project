"""
planning/junction_decider.py — 路口决策器（D-03）

在「路口」这个唯一决策点，做「选目标 + 规划」合一：
一次 Dijkstra 到所有候选，算 score 选目标，取第一跳返回 左/右/直。

设计依据：docs/architecture/边级巡航控制层详细设计.md §6。

核心原则（2026-08-20 拍板）：
  - 路径规划只负责把车导航到「目标节点/端点」，到达后由控制层编排后续动作。
  - 无朝向约束：到达端点后可原地转向，Dijkstra 不需带朝向状态（用现有的
    MapOracle._dijkstra，它已带禁掉头+转向代价，可直接复用）。
  - 选目标与规划合一：一次 Dijkstra 到所有候选，score 选最大者，取第一跳。
  - 死规则：巡逻时 ban 出发区 + 颈通道（由编排层给 blocked/dijkstra 地图源时处理）。

score 体系（全负分扣分制，初始值待仿真调优）：
  score = 距离分(0~-10) + 转向分(-5/次) + 隧道分(-10/条) + 收益分(+10/+30/+10)
"""
from dataclasses import dataclass
from typing import List, Optional, Set, Tuple

from ..domain.line_graph import edge_heading, classify_turn
from .map_oracle import MapOracle


# ================================================================
# 权重配置（初始值，待仿真调优）
# ================================================================
@dataclass
class JunctionDecideWeights:
    """score 权重（全负分扣分制）。初始值，仿真标定后调。"""
    reward_rfid: float = 10.0        # 打卡 RFID 收益
    reward_culvert: float = 30.0     # 侦查涵洞收益
    reward_tunnel_target: float = 10.0  # 通过隧道任务收益
    turn_cost: float = -5.0          # 每次转向代价
    tunnel_cost: float = -10.0       # 每条隧道代价（可调：隧道好搞则调小）
    # 距离归一化：把物理距离 mm 映射到 0 ~ -10 分
    # distance_score = -min(dist / distance_norm_mm, 1.0) * 10
    distance_norm_mm: float = 4000.0  # 地图最长可达距离 ≈ 4m，对应 -10 分


@dataclass
class JunctionDecision:
    """路口决策结果"""
    action: str                     # "straight" / "left" / "right" / "stop"
    target_node: str = ""           # 选中的目标节点
    next_node: str = ""             # 下一路口节点（w1）
    score: float = 0.0


class JunctionDecider:
    """
    路口决策器：选目标 + 规划合一。

    使用方式：
        decider = JunctionDecider(oracle, topo)
        decision = decider.decide(junction, incoming, task_queue, blocked_edges)
    """

    def __init__(self, oracle: MapOracle, topo, weights: JunctionDecideWeights = None):
        self._oracle = oracle
        self._topo = topo
        self._w = weights or JunctionDecideWeights()

    # ================================================================
    # 主入口
    # ================================================================
    def decide(self, junction: str, incoming: float,
               task_queue, blocked_edges: Set[int] = None) -> JunctionDecision:
        """
        在路口 junction 做决策。

        task_queue: 有 pending_rfid() / pending_culverts() 接口的编排层对象。
        blocked_edges: ban 出发区/颈通道/障碍边后的边集合（由编排层传入）。
        incoming: 车驶入 junction 的朝向（world yaw_deg）。
        """
        blocked = blocked_edges or set()
        candidates = self._build_candidates(task_queue)

        best: Optional[JunctionDecision] = None
        best_score = -float("inf")

        for target in candidates:
            score, action, next_node = self._evaluate_target(
                junction, incoming, target, blocked
            )
            if action is None:
                continue                    # 不可达（如被 blocked）
            if score > best_score:
                best_score = score
                best = JunctionDecision(
                    action=action, target_node=target,
                    next_node=next_node, score=score,
                )

        if best is None:
            return JunctionDecision(action="stop")
        return best

    # ================================================================
    # 候选构建
    # ================================================================
    def _build_candidates(self, task_queue) -> List[str]:
        """候选目标节点：未打卡 RFID 中心 + 未侦查涵洞边的两端端口。"""
        candidates: List[str] = []
        for node in task_queue.pending_rfid():
            candidates.append(node)
        for edge_id in task_queue.pending_culverts():
            try:
                edge = self._topo.get_edge_by_id(edge_id)
            except KeyError:
                continue
            candidates.append(edge.node_a)
            candidates.append(edge.node_b)
        return candidates

    # ================================================================
    # 单目标评估
    # ================================================================
    def _evaluate_target(self, junction, incoming, target, blocked) -> Tuple[float, Optional[str], str]:
        """返回 (score, action, next_node)。不可达返回 (..., None, ...)。"""
        dist, path = self._oracle._dijkstra(
            junction, target, blocked_edges=blocked, entry_heading=incoming
        )
        if path is None or len(path) < 2:
            return -float("inf"), None, ""

        # 统计转向次数、隧道条数
        turn_count = self._count_turns(path)
        tunnel_count = self._count_tunnels(path)

        # score 各分量
        distance_score = self._distance_score(dist)
        turn_score = turn_count * self._w.turn_cost
        tunnel_score = tunnel_count * self._w.tunnel_cost
        reward = self._reward(target)

        score = distance_score + turn_score + tunnel_score + reward

        # 第一跳 = path[1]
        next_node = path[1]
        # action 基于「从 junction 朝 next_node」的方向，相对进入朝向 incoming
        out_heading = edge_heading(self._topo, junction, next_node)
        action = classify_turn(incoming, out_heading)

        return score, action, next_node

    def _count_turns(self, path: List[str]) -> int:
        """统计 path 中（非直行的）转弯次数。"""
        turns = 0
        for i in range(len(path) - 2):
            a, b, c = path[i], path[i + 1], path[i + 2]
            h1 = edge_heading(self._topo, a, b)
            h2 = edge_heading(self._topo, b, c)
            kind = classify_turn(h1, h2)
            if kind in ("left", "right"):
                turns += 1
        return turns

    def _count_tunnels(self, path: List[str]) -> int:
        """统计 path 经过的隧道边条数。"""
        count = 0
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            try:
                edge = self._topo.get_edge(a, b)
            except KeyError:
                continue
            if edge.is_tunnel:
                count += 1
        return count

    def _distance_score(self, dist_mm: float) -> float:
        """距离归一化到 0 ~ -10 分。"""
        ratio = min(dist_mm / self._w.distance_norm_mm, 1.0)
        return -ratio * 10.0

    def _reward(self, target: str) -> float:
        """
        目标收益：RFID +10，涵洞 +30，隧道任务 +10。

        TODO(D-03 细化)：当前用节点类型粗判（中心有 RFID→+10，端口→+30），
        无法区分「涵洞端口」vs「隧道任务」vs「无任务的路口端口」。后续需让
        build_candidates 携带「候选类型」，而非靠节点类型猜收益。
        隧道任务目标（+10）暂未区分，需补充。
        """
        node = self._topo.get_node(target)
        if getattr(node, "has_rfid", False):
            return self._w.reward_rfid
        return self._w.reward_culvert
