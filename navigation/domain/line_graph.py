"""
navigation/line_graph.py — 线图（Line Graph）建模 + 转向转移枚举

在「无向端口图」RaceTrackTopology 之上构建「线图」，用于带朝向约束的路径规划。

动机（Task 12 / docs/navigation/优化仿真路径规划算法.md）：
  真车不能原地掉头，只能直行 / 左转 90° / 右转 90°。原图以「节点」为状态，无从
  禁止 180° 掉头；线图把状态升级为「(当前节点, 进入朝向)」，使掉头在转移层被
  直接删除，Dijkstra/TSP 搜索出的每一条路径天然可执行。

线图结构：
  - 线图节点 = 一条有向边 (from_node -> to_node)，由 (from_node, to_node) 标识。
  - 线图转移 (u -> v -> w) = 原图里「先走 u->v，再走 v->w」的连续两步，权重含
    行驶代价 + 在 v 处的转向代价。

约定：
  - 边沿无向，两条有向边 (a,b) 与 (b,a) 都合法（同一物理直道的两个方向）。
  - 朝向 = 世界 yaw_deg，用 atan2(dx, dy) 计算（与 state_machine/_calc_expected_yaw
    、path_planner/_node_seq_to_tasks 保持一致）。0°=Y+，90°=X+。
  - 转向角 = (出边朝向 - 入边朝向) 归一化到 (-180, 180]。
    0°=直行；+90° 左转；-90° 右转；±180°=掉头（禁止）。
"""

import math
from typing import List, Tuple, Optional, Set, Dict

from .topology import RaceTrackTopology, MapEdge, get_topology

try:
    from ... import config as _cfg
except ImportError:
    import config as _cfg


# 转向代价（等价距离 mm），默认值可被 settings.yaml:planner 覆盖
_TURN_COST_STRAIGHT = 0.0
_TURN_COST_90 = _cfg.get("planner.turn_cost_90_mm", 300.0)  # 左/右转 90° 等价距离
_REVERSE_COST = _cfg.get("planner.reverse_cost_mm", 1500.0)  # 倒车 5×，仅障碍响应层


def _normalize_deg(angle: float) -> float:
    """把角度归一化到 (-180, 180]。"""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def edge_heading(topo: RaceTrackTopology, from_node: str, to_node: str) -> float:
    """
    计算有向边 from_node -> to_node 的世界 yaw（deg）。

    与 path_planner/_node_seq_to_tasks、state_machine/_calc_expected_yaw 一致：
    yaw = degrees(atan2(dx, dy))，0°=Y+，90°=X+。
    """
    a = topo.get_node(from_node)
    b = topo.get_node(to_node)
    dx = b.x_mm - a.x_mm
    dy = b.y_mm - a.y_mm
    return math.degrees(math.atan2(dx, dy))


def turn_angle_degs(from_heading: float, to_heading: float) -> float:
    """
    由「进入朝向」转到「离开朝向」的转角（deg），归一化到 (-180, 180]。

    正 = 左转；负 = 右转；0 = 直行；±180（近似）= 掉头。
    """
    return _normalize_deg(to_heading - from_heading)


# 转向分类阈值（deg）：|转角| < STRAIGHT 视为直行，|转角| > UTURN 视为掉头。
_TURN_STRAIGHT_DEG = 45.0
_TURN_UTURN_DEG = 135.0


def classify_turn(from_heading: float, to_heading: float) -> str:
    """
    把转角分类为 'straight' | 'left' | 'right' | 'uturn'。

    端口模型下目标转角只有 0° / ±90° / 180° 四档，阈值取 45°/135° 即可稳健区分：
      - |diff| < 45°  → 直行
      - |diff| > 135° → 掉头
      - 其余按符号 → 左转 / 右转
    """
    diff = turn_angle_degs(from_heading, to_heading)
    if abs(diff) < _TURN_STRAIGHT_DEG:
        return 'straight'
    if abs(diff) > _TURN_UTURN_DEG:
        return 'uturn'
    return 'left' if diff > 0 else 'right'


def turn_cost(from_heading: float, to_heading: float) -> float:
    """
    转向代价（等价距离 mm）。

    - 直行：0
    - 左/右 90°：300mm
    - 掉头：返回 math.inf，表示「不可执行」（由调用方据此跳过该转移）。
    """
    c = classify_turn(from_heading, to_heading)
    if c == 'straight':
        return _TURN_COST_STRAIGHT
    if c in ('left', 'right'):
        return _TURN_COST_90
    return math.inf  # uturn


def iter_out_edges(topo: RaceTrackTopology, node: str,
                   entry_heading: Optional[float] = None,
                   blocked_edges: Optional[Set[int]] = None
                   ) -> List[Tuple[str, float, float]]:
    """
    枚举从 node 出发的可执行「出边」，返回 [(to_node, 出边朝向, 转向代价), ...]。

    若 entry_heading 为 None，不施加转向约束（等价于无向出边，转向代价为 0），
    用于掉头约束尚未生效的兼容场景。

    约束（entry_heading 非 None 时）：
      - 跳过掉头（转向代价为 inf）
      - 跳过 blocked_edges 中的边
      - 其余保留，转向代价按转角给出
    """
    blocked = blocked_edges or set()
    out: List[Tuple[str, float, float]] = []
    for edge in topo.get_neighbors(node):
        if edge.edge_id in blocked:
            continue
        to_node = edge.other(node)
        to_heading = edge_heading(topo, node, to_node)
        if entry_heading is None:
            out.append((to_node, to_heading, 0.0))
        else:
            cost = turn_cost(entry_heading, to_heading)
            if math.isinf(cost):
                continue  # 掉头禁止
            out.append((to_node, to_heading, cost))
    return out


def has_safe_exit(topo: RaceTrackTopology, node: str, entry_heading: float,
                  blocked_edges: Optional[Set[int]] = None) -> bool:
    """
    「后方有无出口」判定（Task 12 §6.2a）。

    车以 entry_heading 倒车回到 node 后，是否存在至少一条可进入边 F：
      - F 非当前障碍边（由 blocked_edges 表达）
      - F 非已封锁边
      - 从 entry_heading 转到 F 的转角 ∈ {直行, ±90°}（可执行）
      - F 可从该恢复姿态驶入

    存在 → R1（可安全后退）；不存在 → R2（必须倒车转弯）。
    """
    outs = iter_out_edges(topo, node, entry_heading, blocked_edges)
    return len(outs) > 0


class LineGraph:
    """
    线图主类：把 RaceTrackTopology 一次性转换成线图，提供图搜索所需的查询接口。

    不改变拓扑数据，只在其上叠加「有向 + 转向」视图。
    """

    def __init__(self, topo: Optional[RaceTrackTopology] = None,
                 turn_cost_90: float = _TURN_COST_90):
        self.topo = topo or get_topology()
        self.turn_cost_90 = turn_cost_90

    # ------------------------------------------
    # 线图实体（有向边）
    # ------------------------------------------

    def line_node(self, from_node: str, to_node: str) -> Tuple[str, str]:
        """有向边标识 = (from_node, to_node)。"""
        return (from_node, to_node)

    def heading(self, from_node: str, to_node: str) -> float:
        return edge_heading(self.topo, from_node, to_node)

    def line_node_heading(self, ln: Tuple[str, str]) -> float:
        """线图节点（有向边）对应的行驶朝向。"""
        return edge_heading(self.topo, ln[0], ln[1])

    # ------------------------------------------
    # 转向转移（线图边）
    # ------------------------------------------

    def out_transitions(self, ln: Tuple[str, str],
                        blocked_edges: Optional[Set[int]] = None
                        ) -> List[Tuple[Tuple[str, str], float]]:
        """
        线图节点 ln=(u,v) 的「出边」（非 0 代价的出转移）：车在 v，进入朝向已固定为
        边 (u,v) 的朝向，可继续转上哪些 (v,w)，返回 [((v,w), 转向代价), ...]。

        掉头 (w == u) 或转向代价为 inf 的转移被排除。
        """
        u, v = ln
        entry = edge_heading(self.topo, u, v)
        result: List[Tuple[Tuple[str, str], float]] = []
        for w, _, cost in iter_out_edges(self.topo, v, entry, blocked_edges):
            if w == u:
                continue  # 回到来路，等价掉头
            result.append(((v, w), cost))
        return result
