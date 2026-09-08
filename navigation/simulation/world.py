"""提供不越权写入导航状态的确定性仿真真值世界。"""

import math
import random
from typing import Iterable, Optional, FrozenSet, Set

from navigation.contracts import (
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

    def __init__(self, topology: Optional[TrackTopology] = None, seed: int = 0,
                 initial_pose: Optional[WorldPose] = None,
                 blocked_edge_ids: Optional[Iterable[str]] = None,
                 culvert_edge_ids: Optional[Iterable[str]] = None) -> None:
        """创建确定性世界；未显式提供真值时由 seed 生成。"""

        self.topology = topology or build_default_topology()
        self.seed = seed
        self._random = random.Random(seed)
        start = self.topology.get_node("START")
        self._pose = initial_pose or WorldPose(start.x_mm, start.y_mm, 90.0)
        external = [edge.edge_id for edge in self._edges() if edge.road_kind != "INTERNAL"]
        if blocked_edge_ids is None:
            blocked_edge_ids = self._random_truth(external, 1)
        blocked_edge_ids = tuple(blocked_edge_ids)
        if culvert_edge_ids is None:
            candidates = [edge for edge in external if edge not in set(blocked_edge_ids)]
            culvert_edge_ids = self._random_truth(candidates, 1)
        culvert_edge_ids = tuple(culvert_edge_ids)
        self._truth_blocked: Set[str] = set(blocked_edge_ids)
        self._truth_culvert: Set[str] = set(culvert_edge_ids)
        self._now = 0.0
        self._observation_count = 0
        self._last_frame_id = None

    def execute_motion(self, command: ExecutionCommand):
        """执行一条运动命令并返回终局；不会写入导航地图或逻辑位姿。"""

        if isinstance(command, DriveExecutionCommand):
            distance = command.distance_mm
            self._translate(distance)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(distance), self._pose, distance)
        if isinstance(command, ReverseExecutionCommand):
            distance = command.distance_mm
            self._translate(-distance)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(distance), self._pose, distance)
        if isinstance(command, TurnExecutionCommand):
            self._turn(command.forward_trajectory_id, 90.0)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(1.0), self._pose, 0.0, self._pose.yaw_deg)
        if isinstance(command, RetraceTurnExecutionCommand):
            self._turn(command.retrace_trajectory_id, -90.0)
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(1.0), self._pose, 0.0, self._pose.yaw_deg)
        if isinstance(command, StopExecutionCommand):
            return SimMotionResult(ExecutionOutcome.COMPLETED, self._advance_time(0.1), self._pose)
        raise TypeError("仿真运动端口不支持命令：{}".format(type(command).__name__))

    def observe(self) -> PerceptionFrame:
        """按当前位置生成相对视觉帧；生成过程不修改 RuntimeMap。"""

        self._observation_count += 1
        self._last_frame_id = "sim-frame-{}".format(self._observation_count)
        branches = self._road_features()
        targets = []
        for edge_id in self._observable_edges():
            if edge_id in self._truth_blocked:
                targets.append(TargetDetection("OBSTACLE", self._relative_region(edge_id), 1.0, True))
            if edge_id in self._truth_culvert:
                targets.append(TargetDetection("CULVERT", self._relative_region(edge_id), 1.0, True))
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

    def _turn(self, trajectory_id, fallback):
        name = str(trajectory_id).lower()
        sign = -1.0 if "left" in name else 1.0 if "right" in name else 0.0
        self._pose = WorldPose(self._pose.x_mm, self._pose.y_mm,
                               (self._pose.yaw_deg + sign * fallback) % 360.0)

    def _nearest_junction(self):
        nodes = [self.topology.get_node(node_id) for node_id in self.topology.node_ids()]
        return min(nodes, key=lambda node: (node.x_mm - self._pose.x_mm) ** 2 + (node.y_mm - self._pose.y_mm) ** 2)

    def _observable_edges(self):
        junction = self._nearest_junction()
        try:
            result = []
            for edge in self.topology.outgoing_cruise_edges(junction.node_id):
                physical = [edge_id for edge_id in edge.physical_edge_ids
                            if self.topology.get_physical_edge(edge_id).road_kind != "INTERNAL"]
                if physical:
                    result.append(physical[0])
            return tuple(result)
        except (KeyError, ValueError):
            return ()

    def _road_features(self):
        junction = self._nearest_junction()
        forward = left = right = False
        for edge in self.topology.outgoing_cruise_edges(junction.node_id):
            target = self.topology.get_node(edge.to_junction)
            bearing = math.degrees(math.atan2(target.y_mm - junction.y_mm, target.x_mm - junction.x_mm)) % 360.0
            delta = (bearing - self._pose.yaw_deg + 540.0) % 360.0 - 180.0
            if abs(delta) < 25.0: forward = True
            elif delta < 0: left = True
            else: right = True
        return RoadFeatures(forward, left, right, 1.0)

    def _relative_region(self, edge_id):
        edge = self.topology.get_physical_edge(edge_id)
        a, b = self.topology.get_node(edge.from_node_id), self.topology.get_node(edge.to_node_id)
        bearing = math.degrees(math.atan2(b.y_mm - a.y_mm, b.x_mm - a.x_mm)) % 360.0
        delta = (bearing - self._pose.yaw_deg + 540.0) % 360.0 - 180.0
        return "FORWARD" if abs(delta) < 45 else "LEFT" if delta < 0 else "RIGHT"
