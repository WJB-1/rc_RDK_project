"""
perception/adapter.py — L1 感知适配层（PerceptionAdapter）

导航内唯一的「适配层」。职责：
  - 输入：外层纯视觉建模给出的 VisualObservation（纯相对事实）
  - 翻译：相对 → 绝对（哪条支路 → 地图 edge_id），加消歧
  - 产出：MapUpdateIntent，交由 RuntimeMap 落库

分界（2026-08-20 拍板）：
  - 外层（perception/ + vision/）做纯视觉建模：形态学推测哪条支路 + 差错控制。
    产出 VisualObservation（相对事实），不碰地图、不碰 edge_id。
  - 本层（navigation 内）做相对→绝对换算 + 消歧 + 批内去重，为地图更新提供数据。
  - 类型 VisualObservation/RelativeLane 定义在 vision/contracts.py（外层视觉侧），
    本层 import 它，依赖方向朝内，外层不反向依赖 navigation。

未落地（后置）：
  - 相对→绝对换算算法（D-01.2）：像素/相对支路 → 地图 edge_id 的空间翻译。
    依赖拓扑 + 绝对位姿，本次只留 translate 接口签名。
"""
from dataclasses import dataclass, field
from typing import Any, List, Optional

try:
    from ...vision.contracts import VisualObservation, RelativeLane
    from ..domain.topology import get_topology
    from ..domain.line_graph import edge_heading, turn_angle_degs
    from ..contracts import MapUpdateIntent
except ImportError:
    from vision.contracts import VisualObservation, RelativeLane
    from navigation.domain.topology import get_topology
    from navigation.domain.line_graph import edge_heading, turn_angle_degs
    from navigation.contracts import MapUpdateIntent


@dataclass
class _LegacyMapUpdateIntent:
    """
    适配层产出的地图更新意图，交由 RuntimeMap 落库。

    update 取值与 RuntimeMap 写入口对应：
      "mark_culvert_discovered" / "block_edge" / "set_culvert_endpoint" / ...
    """
    edge_id: int
    update: str
    value: Any = None


class PerceptionAdapter:
    """
    感知适配器 —— 外层视觉事实 → 地图绝对更新。

    批接口 adapt() 是纯函数：逐条 translate + 批内去重，产出 MapUpdateIntent
    列表，不直接提交；提交动作由 perception 主流程调用 RuntimeMap 写入口完成。
    """

    # 相对支路 → 相对转角（世界坐标系，y 增大=0°、x 增大=+90°）
    # LEFT/RIGHT 是「屏幕图像里的左/右」，加上小车地图朝向 entry_heading，
    # 即可得世界方位：target = entry_heading + 相对转角。
    # 约定（固定，无需标定）：屏幕左 = 车头左 = 世界 +90°；屏幕右 = -90°。
    # 摄像头朝前正装时，屏幕左即车的左侧，为固定 90° 夹角关系。
    _RELATIVE_TURN = {
        RelativeLane.AFTER_JUNCTION: 0.0,
        RelativeLane.LEFT_BRANCH:    90.0,     # 屏幕左 = 世界 +90°
        RelativeLane.RIGHT_BRANCH:  -90.0,      # 屏幕右 = 世界 -90°
    }

    def translate(self, obs: VisualObservation,
                  current_edge_id: int, entry_heading: float) -> Optional[MapUpdateIntent]:
        """
        单条翻译：VisualObservation（相对）→ MapUpdateIntent（绝对）。

        ctx = (current_edge_id, entry_heading) 两个「当前寄存器」值：
          - current_edge_id：当前边（BEFORE_JUNCTION 直接用它）
          - entry_heading：当前朝向（推算 AFTER/LEFT/RIGHT 的前方分叉）
        地图在函数内部直接 get_topology() 查，不塞进参数。

        相对→绝对换算 + 消歧：
          - obstacle → block_edge
          - culvert → mark_culvert_discovered
          - wall → 消歧：换算出边若 is_tunnel 则忽略(None)，否则 mark_culvert_discovered
        """
        # 1. 换算目标 edge_id
        target_edge = self._resolve_edge(obs.relative_lane, current_edge_id, entry_heading)
        if target_edge is None:
            return None                      # 以地图为准：换算不到，忽略

        edge = get_topology().get_edge_by_id(target_edge)

        # 2. 消歧 + object_type → update 映射
        if obs.object_type == "obstacle":
            return MapUpdateIntent(kind="block_edge", edge_id=target_edge)
        if obs.object_type == "culvert":
            return MapUpdateIntent(kind="discover_culvert", edge_id=target_edge)
        if obs.object_type == "wall":
            if edge.is_tunnel:
                return None                  # 隧道侧墙，忽略
            return MapUpdateIntent(kind="discover_culvert", edge_id=target_edge)
        return None                          # 未知 object_type，忽略

    def _resolve_edge(self, relative_lane: RelativeLane,
                      current_edge_id: int, entry_heading: float) -> Optional[int]:
        """相对支路 → 目标 edge_id 的换算核心。"""
        topo = get_topology()

        # BEFORE：当前边直接就是答案（视觉已映射好，零查找）
        if relative_lane == RelativeLane.BEFORE_JUNCTION:
            return current_edge_id

        edge = topo.get_edge_by_id(current_edge_id)
        if edge.is_internal:
            return None                      # 当前在内部半边，无法推前方分叉

        # 前方端口：车沿 edge 朝 entry_heading 开，前端是 edge 的哪一端
        toward_b = self._heading_matches(entry_heading,
                                         edge_heading(topo, edge.node_a, edge.node_b))
        front_port = edge.node_b if toward_b else edge.node_a

        # 前方路口中心 = 从 front_port 再经内部半边跳一跳，到达方块中心
        front_center = None
        for e in topo.get_neighbors(front_port):
            if e.is_internal and e.other(front_port) != edge.node_b and e.other(front_port) != edge.node_a:
                front_center = e.other(front_port)
                break
        if front_center is None:
            return None                      # 前端不是端口，无法定位路口中心

        # 目标绝对朝向 = entry_heading + 相对转角
        target_heading = self._normalize(entry_heading + self._RELATIVE_TURN[relative_lane])

        # 查 front_center 的 port，找中心方位 == target_heading 的 port，再跳一跳拿直道
        for port_edge in topo.get_neighbors(front_center):
            if not port_edge.is_internal:
                continue
            port = port_edge.other(front_center)
            port_heading = edge_heading(topo, front_center, port)
            if self._heading_matches(port_heading, target_heading):
                # 从 port 连出去的非 internal 直道 = 目标边
                for straight_edge in topo.get_neighbors(port):
                    if straight_edge.is_internal:
                        continue
                    if straight_edge.other(port) == front_center:
                        continue
                    return straight_edge.edge_id
        return None                          # 该方向无出口（如 T 字路口无直行 AFTER）

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle

    @staticmethod
    def _heading_matches(h1: float, h2: float, tol: float = 30.0) -> bool:
        """两朝向是否匹配（考虑 ±180° 等价），容差 30°。"""
        diff = turn_angle_degs(h1, h2)
        return abs(diff) <= tol

    def adapt(self, observations: List[VisualObservation],
              current_edge_id: int, entry_heading: float) -> List[MapUpdateIntent]:
        """
        批封装：逐条 translate + 批内去重（同 update 类型 + 同 edge_id 只留一次）。

        纯函数，不提交；提交由 perception 主流程做，且由 RuntimeMap 写入口做
        跨批幂等兜底（双向冲突保护）。
        """
        intents: List[MapUpdateIntent] = []
        seen = set()
        for obs in observations:
            intent = self.translate(obs, current_edge_id, entry_heading)
            if intent is None:
                continue
            key = (intent.kind, intent.edge_id)
            if key in seen:
                continue          # 批内重复，静默丢弃
            seen.add(key)
            intents.append(intent)
        return intents
