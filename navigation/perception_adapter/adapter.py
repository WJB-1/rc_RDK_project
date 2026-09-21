"""将视觉相对事实翻译为绝对边观察，不直接修改导航状态。"""

from typing import Dict

from navigation.contracts import (
    CoverageInterval,
    EdgeObservation,
    EdgePassability,
    PerceptionFrame,
    PerceptionOutcome,
    PerceptionTranslation,
)
from navigation.domain import AtNode, OnCruiseEdge, TrackTopology
from .mapping import map_region_to_traversal
from .visibility import PerceptionVisibilityProfile


class PerceptionAdapter:
    """感知适配器公共门面，读取导航状态并输出原子观察事实。"""

    def __init__(self, state_store, topology: TrackTopology, visibility_profile=None) -> None:
        """注入只读状态、静态拓扑和待标定可见性配置。"""
        self._state_store = state_store
        self._topology = topology
        self._visibility = visibility_profile or PerceptionVisibilityProfile()

    def translate(self, frame: PerceptionFrame) -> PerceptionTranslation:
        """将单帧视觉结果翻译为绝对边观察，失败时整帧原子地返回不确定。"""
        state = self._state_store.robot_state()
        if not isinstance(state.location, (AtNode, OnCruiseEdge)):
            return PerceptionTranslation(frame.frame_id, PerceptionOutcome.INCONCLUSIVE, reason="当前位置无法解析视觉方向")
        blocked_edge_ids = self._state_store.runtime_map_snapshot().blocked_edge_ids
        observations: Dict[str, EdgeObservation] = {}
        for target in frame.targets:
            if not target.valid:
                continue
            region = target.relative_region
            if region in ("LEFT_BRANCH", "LEFT", "JUNCTION_LEFT_BRANCH") and not frame.road_features.has_left_branch:
                return PerceptionTranslation(frame.frame_id, PerceptionOutcome.INCONCLUSIVE, reason="左侧目标与道路特征矛盾")
            if region in ("RIGHT_BRANCH", "RIGHT", "JUNCTION_RIGHT_BRANCH") and not frame.road_features.has_right_branch:
                return PerceptionTranslation(frame.frame_id, PerceptionOutcome.INCONCLUSIVE, reason="右侧目标与道路特征矛盾")
            if region in ("FORWARD_BRANCH", "FORWARD", "JUNCTION_FORWARD_BRANCH") and not frame.road_features.has_forward_branch:
                return PerceptionTranslation(frame.frame_id, PerceptionOutcome.INCONCLUSIVE, reason="前方目标与道路特征矛盾")
            traversal_id = map_region_to_traversal(state, self._topology, region)
            if traversal_id is None:
                return PerceptionTranslation(frame.frame_id, PerceptionOutcome.INCONCLUSIVE, reason="无法唯一映射视觉目标到绝对边")
            edge = self._topology.get_cruise_edge(*traversal_id.split("->", 1))
            edge_id = next(
                physical_edge_id
                for physical_edge_id in edge.physical_edge_ids
                if self._topology.get_physical_edge(physical_edge_id).road_kind != "INTERNAL"
            )
            current = observations.get(edge_id)
            culvert_found = target.kind == "CULVERT" or (target.kind == "WALL" and edge.road_kind != "TUNNEL")
            # 未发现涵洞且目标不是 WALL 时，把整条边都记为"已检查过涵洞"。
            # 这样 EventProjector 会推 CONFIRM_NO_CULVERT（覆盖 0~1），
            # 地图快照就会把该边标为 CONFIRMED_ABSENT，TargetSelector 里
            # 现成的 all(... is CONFIRMED_ABSENT) 过滤才会命中，避免机器人
            # 在 N7 → T3_L → T2_L → N10 → N9 → N8 → N7 这类环上无限循环。
            if culvert_found or target.kind == "WALL":
                coverage = ()
            else:
                coverage = (CoverageInterval(0.0, 1.0),)
            if target.kind == "OBSTACLE" or (current and current.passability is EdgePassability.BLOCKED):
                passability = EdgePassability.BLOCKED
            elif isinstance(state.location, AtNode) and edge_id not in blocked_edge_ids:
                passability = EdgePassability.CLEAR
            else:
                passability = current.passability if current else EdgePassability.UNKNOWN
            observations[edge_id] = EdgeObservation(
                edge_id=edge_id,
                passability=passability,
                culvert_found=culvert_found,
                no_culvert_coverage=coverage,
                obstacle_distance_mm=getattr(target, "distance_mm", None),
                observation_scope=(
                    "junction_full"
                    if isinstance(state.location, AtNode) and passability is EdgePassability.CLEAR
                    else "observation_zone"
                ),
            )
        # 没有目标也可能构成可靠负证据：路口确认道路无障碍，同时把整条边记为已检查过涵洞。
        if isinstance(state.location, AtNode):
            for region, present in (
                ("FORWARD_BRANCH", frame.road_features.has_forward_branch),
                ("LEFT_BRANCH", frame.road_features.has_left_branch),
                ("RIGHT_BRANCH", frame.road_features.has_right_branch),
            ):
                if not present:
                    continue
                traversal_id = map_region_to_traversal(state, self._topology, region)
                if traversal_id is None:
                    continue
                edge = self._topology.get_cruise_edge(*traversal_id.split("->", 1))
                edge_id = next(
                    physical_edge_id
                    for physical_edge_id in edge.physical_edge_ids
                    if self._topology.get_physical_edge(physical_edge_id).road_kind != "INTERNAL"
                )
                if edge_id not in observations and edge_id not in blocked_edge_ids:
                    observations[edge_id] = EdgeObservation(
                        edge_id=edge_id,
                        passability=EdgePassability.CLEAR,
                        no_culvert_coverage=(CoverageInterval(0.0, 1.0),),
                        observation_scope="junction_full",
                    )
        return PerceptionTranslation(frame.frame_id, PerceptionOutcome.CONFIRMED, tuple(observations.values()))