"""
navigation/vision_checker.py — 视觉模拟器

模拟小车行驶过程中的视觉发现:
1. 同边发现: 小车在边上行驶，进入可视范围后"看到"障碍物/涵洞
2. 路口侧视: 小车到达路口节点时，可"看到"相邻边上的涵洞（不包含障碍物）

设计理念:
- 障碍物只能在同边行驶时发现（距离 < visible_range）
- 涵洞可在同边行驶或路口侧视时发现
- 已发现的对象不会重复触发
"""

from typing import List, Union

from ...contracts import CulvertEvent, ObstacleEvent, CulvertType
from .scene import SimScene
from ...domain.topology import RaceTrackTopology


class VisionChecker:
    """
    视觉模拟检查器

    每个 tick 调用 check_on_edge()（如果在边上行驶）
    每个节点到达时调用 check_at_node()（路口侧视）
    """

    def __init__(
        self,
        topo: RaceTrackTopology,
        scene: SimScene,
        visible_range_mm: float = 400.0,
    ):
        self._topo = topo
        self._scene = scene
        self._visible_range_mm = visible_range_mm

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def check_on_edge(
        self, car_position_mm: float, edge_id: int, from_node: str
    ) -> list:
        """
        检查当前边上是否有进入可视范围的障碍物/涵洞。

        Args:
            car_position_mm: 从 from_node 沿边行驶的距离 (mm)
            edge_id: 当前所在的边 ID
            from_node: 行驶起始节点

        Returns:
            list: 发现的事件列表（ObstacleEvent 或 CulvertEvent）

        注意：scene 的 offset 是相对边的 node_a 计量的。若 from_node 是
        node_b（从对端进入），需把 car_position_mm 换算成"距 node_a 的距离"
        （= 边长 - car_position_mm），否则 offset 比对会错位，导致漏检。
        """
        events = []

        # 换算：scene offset 相对 node_a，car_pos 相对 from_node
        edge = self._topo.get_edge_by_id(edge_id)
        from_node_b = (from_node == edge.node_b)
        pos_from_a = car_position_mm
        if from_node_b:
            # 从对端进入，距 node_a = 边长 - 距 node_b
            pos_from_a = edge.distance_mm - car_position_mm

        # 检查障碍物（仅在未发现时）
        if (
            edge_id in self._scene.obstacle_edge_ids
            and edge_id not in self._scene.discovered_obstacles
        ):
            offset = self._scene.obstacle_offsets[edge_id]
            # 障碍物在前方：取决于行驶方向
            #   - 从 node_a 进入（pos_from_a 递增）：前方 = pos_from_a < offset
            #   - 从 node_b 进入（pos_from_a 递减）：前方 = pos_from_a > offset
            if from_node_b:
                dist = pos_from_a - offset
                is_ahead = dist > 0.0
            else:
                dist = offset - pos_from_a
                is_ahead = dist > 0.0
            if is_ahead and dist < self._visible_range_mm:
                self._scene.discovered_obstacles.add(edge_id)
                events.append(
                    ObstacleEvent(
                        distance_mm=dist,
                        confidence=1.0,
                    )
                )

        # 检查涵洞（仅在未发现时）
        if (
            edge_id in self._scene.culvert_edge_ids
            and edge_id not in self._scene.discovered_culverts
        ):
            offset = self._scene.culvert_offsets[edge_id]
            dist = abs(offset - pos_from_a)
            if dist < self._visible_range_mm:
                self._scene.discovered_culverts.add(edge_id)
                events.append(
                    CulvertEvent(
                        culvert_type=CulvertType.SIDE,
                        local_x_mm=0.0,
                        local_y_mm=offset - pos_from_a,
                        confidence=1.0,
                    )
                )

        return events

    def check_at_node(self, node_name: str) -> list:
        """
        在路口节点处侧视检查相邻边上的涵洞。

        Args:
            node_name: 当前所在节点名

        Returns:
            list: 发现的 CulvertEvent 列表（不含障碍物）
        """
        events = []

        for edge in self._topo.get_neighbors(node_name):
            if (
                edge.edge_id in self._scene.culvert_edge_ids
                and edge.edge_id not in self._scene.discovered_culverts
            ):
                self._scene.discovered_culverts.add(edge.edge_id)
                events.append(
                    CulvertEvent(
                        culvert_type=CulvertType.SIDE,
                        local_x_mm=0.0,
                        local_y_mm=0.0,
                        confidence=1.0,
                    )
                )

        return events

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------

    def get_discovery_stats(self) -> dict:
        """返回发现进度统计"""
        return {
            "discovered_culverts": len(self._scene.discovered_culverts),
            "recon_culverts": len(self._scene.recon_culverts),
            "total_culverts": len(self._scene.culvert_edge_ids),
            "discovered_obstacles": len(self._scene.discovered_obstacles),
            "total_obstacles": len(self._scene.obstacle_edge_ids),
        }
