"""
domain/runtime_map.py — 运行时地图事实（RuntimeMap）

导航状态唯一权威，单一写入者。收编原 AgentStateMachine 门面平铺的裸 set
（visited_nodes / blocked_edges / discovered_culverts / recon_culverts /
_culvert_targets），并提供带语义的单一写入口。

与静态 topology 的关系：
  - topology（静态）回答「有什么路、路是什么形状」，赛前确定、不可变。
  - RuntimeMap（运行时）回答「这条路现在能不能走、探索到什么程度」，随运行变化。
  两者靠 edge_id / node_name 关联。

设计原则（2026-08-20 拍板）：
  - 地图层只给接口：culvert_endpoint 只声明「需要知道涵洞靠哪端」，
    值如何从视觉/IPM 算出来是感知层的事，本次不填值。
  - 直接存节点名（非 A/B 间接标签）；None = 未测出，END_MIDDLE = 中间。
  - 只存「已发现涵洞」的边。

后续债务（见 docs/debt/8.18_debt.md D-11 剩余）：
  置信度三态（unknown/suspected/confirmed）+ 版本号 + 完全封死单写。
"""
from typing import Optional, Dict, Set

from ..contracts.commands import MapUpdateIntent, RuntimeMapSnapshot

# 哨兵常量（避免魔法字符串）
END_MIDDLE = "MIDDLE"   # 涵洞在边中间，不靠任何一端


class RuntimeMap:
    """运行时地图事实 —— 导航状态唯一权威，单一写入者"""

    def __init__(self):
        # 任务进度（私有，唯一 mutate 经下面的写入口）
        self._visited_nodes: Set[str] = set()          # 已打卡 RFID 节点
        self._blocked_edges: Set[int] = set()          # 障碍封锁的 edge_id
        self._discovered_culverts: Set[int] = set()    # 已发现（未侦查）涵洞 edge_id
        self._recon_culverts: Set[int] = set()         # 已侦查涵洞 edge_id
        self._culvert_targets: Set[int] = set()        # 需侦查的涵洞目标（仿真注入）

        # 涵洞方位（相对边的端点，决策层决定进入方式用）
        # 只存「已发现涵洞」的边；取值：
        #   None          → 发现了但方位尚未测出
        #   END_MIDDLE    → 在边中间，不靠任何一端
        #   "N2" 等节点名  → 靠近该节点（该路口）那一端
        self._culvert_endpoint: Dict[int, Optional[str]] = {}
        self._version = 0

    def snapshot(self) -> RuntimeMapSnapshot:
        return RuntimeMapSnapshot(
            visited_nodes=frozenset(self._visited_nodes),
            blocked_edges=frozenset(self._blocked_edges),
            discovered_culverts=frozenset(self._discovered_culverts),
            recon_culverts=frozenset(self._recon_culverts),
            culvert_targets=frozenset(self._culvert_targets),
            culvert_endpoints=tuple(sorted(self._culvert_endpoint.items())),
            version=self._version,
        )

    def apply(self, intent: MapUpdateIntent) -> bool:
        """Apply one event-shaped fact update and advance the map version."""
        changed = False
        if intent.kind == "block_edge" and intent.edge_id is not None:
            changed = self.block_edge(intent.edge_id)
        elif intent.kind in {"discover_culvert", "mark_culvert_discovered"} and intent.edge_id is not None:
            changed = self.mark_culvert_discovered(intent.edge_id)
        elif intent.kind == "recon_culvert" and intent.edge_id is not None:
            before = intent.edge_id in self._recon_culverts
            self.mark_culvert_reconed(intent.edge_id)
            changed = not before
        elif intent.kind == "visit_node" and intent.node_name:
            before = intent.node_name in self._visited_nodes
            self.mark_rfid_visited(intent.node_name)
            changed = not before
        elif intent.kind == "culvert_endpoint" and intent.edge_id is not None:
            endpoint = intent.value if intent.value is None or isinstance(intent.value, str) else str(intent.value)
            before = self._culvert_endpoint.get(intent.edge_id, object())
            self.set_culvert_endpoint(intent.edge_id, endpoint)
            changed = before != endpoint
        elif intent.kind == "set_culvert_targets" and isinstance(intent.value, (set, frozenset, tuple, list)):
            targets = set(int(value) for value in intent.value)
            changed = targets != self._culvert_targets
            self.set_culvert_targets(targets)
        if changed:
            self._version += 1
        return changed

    # ================================================================
    # 只读视图（返回 frozenset 副本，外界无法修改内部状态）
    # ================================================================

    @property
    def visited_nodes(self) -> frozenset:
        return frozenset(self._visited_nodes)

    @property
    def blocked_edges(self) -> frozenset:
        return frozenset(self._blocked_edges)

    @property
    def discovered_culverts(self) -> frozenset:
        return frozenset(self._discovered_culverts)

    @property
    def recon_culverts(self) -> frozenset:
        return frozenset(self._recon_culverts)

    @property
    def culvert_targets(self) -> frozenset:
        return frozenset(self._culvert_targets)

    # ================================================================
    # 单一写入入口（P0 收口的最终形态）
    # ================================================================

    def mark_culvert_discovered(self, edge_id: int) -> bool:
        """
        标记某条边「发现涵洞」（未侦查）。仅写发现态，不写侦查态。

        幂等 + 双向冲突保护：已标过返回 True（幂等忽略）；若已标障碍边返回
        False（冲突：障碍边与涵洞边永久互斥）。返回 False 让上游知悉。
        """
        if edge_id in self._discovered_culverts:
            return True                       # 幂等：已标过
        if edge_id in self._blocked_edges:
            return False                      # 冲突：障碍边禁标涵洞
        self._discovered_culverts.add(edge_id)
        return True

    def mark_culvert_reconed(self, edge_id: int):
        """标记某条边「涵洞侦查完成」（recon ⊆ discovered）。"""
        self._discovered_culverts.add(edge_id)
        self._recon_culverts.add(edge_id)

    def set_culvert_endpoint(self, edge_id: int, endpoint: Optional[str]):
        """
        设置涵洞方位（只声明接口，本次不填值；感知层后续填）。
        endpoint: None=未测出 / END_MIDDLE=中间 / 节点名=靠该端
        """
        self._culvert_endpoint[edge_id] = endpoint

    def set_culvert_targets(self, targets: Set[int]):
        """注入需侦查的涵洞目标集合（仿真装配用）。"""
        self._culvert_targets = set(targets)

    def block_edge(self, edge_id: int) -> bool:
        """
        标记某条边被障碍封锁（硬约束）。

        幂等 + 双向冲突保护：已标过返回 True（幂等忽略）；若已标涵洞边返回
        False（冲突：涵洞边禁标障碍，硬规则）。障碍永久不可清除。
        """
        if edge_id in self._blocked_edges:
            return True                       # 幂等：已标过
        if edge_id in self._discovered_culverts:
            return False                      # 冲突：涵洞边禁标障碍（硬规则）
        self._blocked_edges.add(edge_id)
        return True

    def mark_rfid_visited(self, node_name: str):
        """标记某 RFID 节点已打卡。"""
        self._visited_nodes.add(node_name)

    # ================================================================
    # 查询
    # ================================================================

    def get_culvert_endpoint(self, edge_id: int) -> Optional[str]:
        """返回涵洞方位（None=未测出 / END_MIDDLE=中间 / 节点名=靠该端）。"""
        return self._culvert_endpoint.get(edge_id)

    def all_culverts_reconed(self) -> bool:
        """是否所有目标涵洞都已侦查完成（读 recon，非 discovered）。"""
        if not self._culvert_targets:
            return True  # 无目标时恒满足
        return self._culvert_targets.issubset(self._recon_culverts)
