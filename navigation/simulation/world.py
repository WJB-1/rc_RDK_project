"""提供不越权写入导航状态的确定性仿真真值世界。"""

import math
import random
from typing import Dict, Iterable, Optional, FrozenSet, Set

from navigation.contracts import (
    CorrectExecutionCommand,
    DriveExecutionCommand, ExecuteTaskExecutionCommand, ExecutionCommand,
    ExecutionOutcome, ObserveExecutionCommand, ReverseExecutionCommand,
    RetraceTurnExecutionCommand, StopExecutionCommand, TurnExecutionCommand,
    PerceptionFrame, RoadFeatures, TargetDetection,
)
from navigation.domain import TrackTopology, build_default_topology
from navigation.domain.state import WorldPose
from .snapshot import SimMotionResult, SimTaskResult, SimWorldSnapshot


class SimWorld:
    """持有静态赛道、连续位姿和障碍/涵洞真值，但不持有 RuntimeMap。"""

    JUNCTION_NEAR_RATIO = 0.35

    def __init__(self, topology: Optional[TrackTopology] = None, seed: int = 0,
                 initial_pose: Optional[WorldPose] = None,
                 blocked_edge_ids: Optional[Iterable[str]] = None,
                 culvert_edge_ids: Optional[Iterable[str]] = None,
                 culvert_positions_by_edge: Optional[Dict[str, float]] = None,
                 obstacle_positions_by_edge: Optional[Dict[str, float]] = None) -> None:
        """创建确定性世界；未显式提供真值时由 seed 生成。"""

        self.topology = topology or build_default_topology()
        self.seed = seed
        self._random = random.Random(seed)
        start = self.topology.get_node("START")
        self._pose = initial_pose or WorldPose(start.x_mm, start.y_mm, 90.0)
        external = [edge.edge_id for edge in self._edges() if edge.road_kind != "INTERNAL"]
        if blocked_edge_ids is None:
            blocked_edge_ids = self._random_truth(external, 3)
        blocked_edge_ids = tuple(blocked_edge_ids)
        if culvert_edge_ids is None:
            candidates = [edge for edge in external if edge not in set(blocked_edge_ids)]
            culvert_edge_ids = self._random_truth(candidates, 8)
        culvert_edge_ids = tuple(culvert_edge_ids)
        self._truth_blocked: Set[str] = set(blocked_edge_ids)
        self._truth_culvert: Set[str] = set(culvert_edge_ids)
        self._culvert_positions = {
            edge_id: (culvert_positions_by_edge or {}).get(edge_id, self._random.random())
            for edge_id in sorted(self._truth_culvert)
        }
        self._obstacle_positions = {
            edge_id: (obstacle_positions_by_edge or {}).get(edge_id, self._random.random())
            for edge_id in sorted(self._truth_blocked)
        }
        self._now = 0.0
        self._observation_count = 0
        self._last_frame_id = None

    def execute_motion(self, command: ExecutionCommand):
        """执行一条运动命令并返回终局；不会写入导航地图或逻辑位姿。"""

        if isinstance(command, CorrectExecutionCommand):
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(1.0), self._pose, 0.0, self._pose.yaw_deg)
        if isinstance(command, DriveExecutionCommand):
            distance = command.distance_mm
            self._translate(distance)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(distance), self._pose, distance)
        if isinstance(command, ReverseExecutionCommand):
            distance = command.distance_mm
            self._translate(-distance)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(distance), self._pose, distance)
        if isinstance(command, TurnExecutionCommand):
            self._turn(command.forward_trajectory_id, 90.0, command.target_traversal_id)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(1.0), self._pose, 0.0, self._pose.yaw_deg)
        if isinstance(command, RetraceTurnExecutionCommand):
            self._turn(command.retrace_trajectory_id, -90.0)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(1.0), self._pose, 0.0, self._pose.yaw_deg)
        if isinstance(command, StopExecutionCommand):
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(0.1), self._pose)
        raise TypeError("仿真运动端口不支持命令：{}".format(type(command).__name__))

    def observe(self, scope=None, traversal_id=None) -> PerceptionFrame:
        """按当前位置生成相对视觉帧；生成过程不修改 RuntimeMap。"""

        self._observation_count += 1
        self._last_frame_id = "sim-frame-{}".format(self._observation_count)
        branches = self._road_features(scope, traversal_id)
        targets = []
        current_road = self._current_road_edge()
        for edge_id, region in self._observable_targets(scope, traversal_id):
            if edge_id in self._truth_blocked:
                targets.append(TargetDetection("OBSTACLE", region, 1.0, True))
            if edge_id in self._truth_culvert:
                targets.append(TargetDetection("CULVERT", region, 1.0, True))
        return PerceptionFrame(self._last_frame_id, self._now, branches, tuple(targets))

    def execute_task(self, command: ExecuteTaskExecutionCommand) -> SimTaskResult:
        """虚拟任务系统默认成功完成已提交任务。"""

        if not isinstance(command, ExecuteTaskExecutionCommand):
            raise TypeError("仿真任务端口只支持 ExecuteTaskExecutionCommand")
        return SimTaskResult(ExecutionOutcome.COMPLETED, self._advance_time(1.0), "completed")

    def snapshot(self) -> SimWorldSnapshot:
        """返回真值世界的不可变副本。"""

        return SimWorldSnapshot(self.seed, self._now, self._pose,
                                frozenset(self._truth_blocked), frozenset(self._truth_culvert),
                                self._observation_count, self._last_frame_id)

    def _edges(self):
        return tuple(self.topology._edges_by_id.values())

    def _random_truth(self, candidates, count):
        if not candidates:
            return ()
        return tuple(self._random.sample(list(candidates), min(count, len(candidates))))

    def _advance_time(self, amount):
        self._now += max(0.0, float(amount)) / 1000.0
        return self._now

    def _translate(self, distance):
        radians = math.radians(self._pose.yaw_deg)
        self._pose = WorldPose(self._pose.x_mm + distance * math.cos(radians),
                               self._pose.y_mm + distance * math.sin(radians), self._pose.yaw_deg)

    def _turn(self, trajectory_id, fallback, target_traversal_id=None):
        """转弯完成后车尾吸附到目标巡航边在当前路口一侧的端口。"""
        name = str(trajectory_id).lower()
        sign = 1.0 if "left" in name else -1.0 if "right" in name else 0.0
        yaw_deg = (self._pose.yaw_deg + sign * fallback) % 360.0
        if target_traversal_id is None:
            self._pose = WorldPose(self._pose.x_mm, self._pose.y_mm, yaw_deg)
            return

        from_node_id, to_node_id = target_traversal_id.split("->", 1)
        cruise_edge = self.topology.get_cruise_edge(from_node_id, to_node_id)
        first_edge = self.topology.get_physical_edge(cruise_edge.physical_edge_ids[0])
        source_endpoint_id = next(
            node_id for node_id in (first_edge.from_node_id, first_edge.to_node_id)
            if node_id != from_node_id and node_id.startswith(from_node_id + ".P_")
        )
        endpoint = self.topology.get_node(source_endpoint_id)
        self._pose = WorldPose(endpoint.x_mm, endpoint.y_mm, yaw_deg)
    def _nearest_junction(self):
        nodes = [self.topology.get_node(node_id) for node_id in self.topology.node_ids()]
        return min(nodes, key=lambda node: (node.x_mm - self._pose.x_mm) ** 2 + (node.y_mm - self._pose.y_mm) ** 2)

    def _observable_edges(self, scope=None, traversal_id=None):
        return tuple(edge_id for edge_id, _ in self._observable_targets(scope, traversal_id))

    def _observable_targets(self, scope, traversal_id):
        current_road = self._current_road_edge()
        if scope == "observation_zone" and current_road is not None:
            current_traversal = traversal_id or current_road[1]
            front_junction = current_traversal.split("->", 1)[1]
            result = []
            for edge_id in self._edges_near_junction(front_junction):
                if edge_id == current_road[2]:
                    if self._target_is_near_junction(edge_id, front_junction, current_traversal):
                        result.append((edge_id, "CURRENT_LANE"))
                    continue
                region = self._junction_region(front_junction, edge_id)
                if region is not None and self._target_is_near_junction(edge_id, front_junction):
                    result.append((edge_id, region))
            return tuple(result)
        if current_road is not None:
            return ((current_road[2], "CURRENT_LANE"),)
        junction = self._nearest_junction()
        facing = self._facing_edge(junction.node_id)
        if facing is None:
            return ()
        return ((facing, "FORWARD"),)

    def _edges_near_junction(self, junction_id):
        result = []
        for edge in self._edges():
            if edge.road_kind == "INTERNAL":
                continue
            if edge.from_node_id == junction_id or edge.to_node_id == junction_id:
                result.append(edge.edge_id)
            elif edge.from_node_id.startswith(junction_id + ".P_") or edge.to_node_id.startswith(junction_id + ".P_"):
                result.append(edge.edge_id)
        return tuple(result)

    def _target_is_near_junction(self, edge_id, junction_id, traversal_id=None):
        edge = self.topology.get_physical_edge(edge_id)
        ratio = self._culvert_positions.get(edge_id, self._obstacle_positions.get(edge_id, 0.5))
        near_ratio = ratio if self._belongs_to_junction(edge.from_node_id, junction_id) else 1.0 - ratio
        if traversal_id is not None and edge_id == self._current_road_edge()[2]:
            near_ratio = 1.0 - ratio if traversal_id.endswith("->" + junction_id) == False else ratio
        return near_ratio <= self.JUNCTION_NEAR_RATIO

    @staticmethod
    def _belongs_to_junction(node_id, junction_id):
        return node_id == junction_id or node_id.startswith(junction_id + ".P_")

    def _junction_region(self, junction_id, edge_id):
        for cruise_edge in self.topology.outgoing_cruise_edges(junction_id):
            if edge_id not in cruise_edge.physical_edge_ids:
                continue
            target = self.topology.get_node(cruise_edge.to_junction)
            origin = self.topology.get_node(junction_id)
            bearing = math.degrees(math.atan2(target.y_mm - origin.y_mm, target.x_mm - origin.x_mm))
            delta = (bearing - self._pose.yaw_deg + 180.0) % 360.0 - 180.0
            if abs(delta) <= 45.0:
                return "JUNCTION_FORWARD_BRANCH"
            return "JUNCTION_RIGHT_BRANCH" if delta < 0 else "JUNCTION_LEFT_BRANCH"
        return None

    def _facing_edge(self, junction_id):
        candidates = []
        origin = self.topology.get_node(junction_id)
        for edge in self.topology.outgoing_cruise_edges(junction_id):
            target = self.topology.get_node(edge.to_junction)
            bearing = math.degrees(math.atan2(target.y_mm - origin.y_mm, target.x_mm - origin.x_mm))
            delta = (bearing - self._pose.yaw_deg + 180.0) % 360.0 - 180.0
            if abs(delta) <= 45.0:
                physical = next((item for item in edge.physical_edge_ids if self.topology.get_physical_edge(item).road_kind != "INTERNAL"), None)
                if physical is not None:
                    candidates.append((abs(delta), edge.traversal_id, physical))
        return min(candidates)[2] if candidates else None

    def _road_features(self, scope=None, traversal_id=None):
        """按标准数学坐标系归类前后左右：delta<0 为顺时针（右），delta>0 为逆时针（左）。"""
        current_road = self._current_road_edge()
        if scope == "observation_zone" and current_road is not None:
            front_junction = (traversal_id or current_road[1]).split("->", 1)[1]
            regions = {
                self._junction_region(front_junction, edge_id)
                for edge_id in self._edges_near_junction(front_junction)
            }
            return RoadFeatures(
                "JUNCTION_FORWARD_BRANCH" in regions,
                "JUNCTION_LEFT_BRANCH" in regions,
                "JUNCTION_RIGHT_BRANCH" in regions,
                1.0,
            )
        if current_road is not None:
            return RoadFeatures(True, False, False, 1.0)
        junction = self._nearest_junction()
        return RoadFeatures(self._facing_edge(junction.node_id) is not None, False, False, 1.0)

    def _relative_region(self, edge_id):
        """按标准数学坐标系返回相对区域：delta<0 为右，delta>0 为左。"""
        junction = self._nearest_junction()
        a = junction
        b = None
        for cruise_edge in self.topology.outgoing_cruise_edges(junction.node_id):
            if edge_id in cruise_edge.physical_edge_ids:
                b = self.topology.get_node(cruise_edge.to_junction)
                break
        if b is None:
            edge = self.topology.get_physical_edge(edge_id)
            a, b = self.topology.get_node(edge.from_node_id), self.topology.get_node(edge.to_node_id)
        bearing = math.degrees(math.atan2(b.y_mm - a.y_mm, b.x_mm - a.x_mm)) % 360.0
        delta = (bearing - self._pose.yaw_deg + 540.0) % 360.0 - 180.0
        return "FORWARD" if abs(delta) < 45 else "RIGHT" if delta < 0 else "LEFT"

    def _current_road_edge(self):
        """按连续位姿将机器人匹配到最近的外部物理道路段。"""
        candidates = []
        for cruise_edge in self._all_cruise_edges():
            for edge_id in cruise_edge.physical_edge_ids:
                physical = self.topology.get_physical_edge(edge_id)
                if physical.road_kind == "INTERNAL":
                    continue
                start = self.topology.get_node(physical.from_node_id)
                end = self.topology.get_node(physical.to_node_id)
                dx = end.x_mm - start.x_mm
                dy = end.y_mm - start.y_mm
                length_sq = dx * dx + dy * dy
                if length_sq <= 0.0:
                    continue
                projection = ((self._pose.x_mm - start.x_mm) * dx + (self._pose.y_mm - start.y_mm) * dy) / length_sq
                if projection < -0.05 or projection > 1.05:
                    continue
                clamped = min(1.0, max(0.0, projection))
                nearest_x = start.x_mm + clamped * dx
                nearest_y = start.y_mm + clamped * dy
                distance_sq = (self._pose.x_mm - nearest_x) ** 2 + (self._pose.y_mm - nearest_y) ** 2
                candidates.append((distance_sq, cruise_edge.traversal_id, edge_id))
        if not candidates:
            return None
        return min(candidates, key=lambda item: (item[0], item[1]))

    def _all_cruise_edges(self):
        for node_id in self.topology.node_ids():
            for edge in self.topology.outgoing_cruise_edges(node_id):
                yield edge
