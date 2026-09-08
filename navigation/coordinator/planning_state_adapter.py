"""把 Coordinator 持有的状态组合成规划器可读的快照端口。"""

from navigation.domain import TaskLifecycle
from navigation.planning import PlanningPhase
from .states import CoordinatorState


class CoordinatorPlanningReadAdapter:
    """只读转发机器人、地图、任务和当前阶段，不暴露任何写入方法。"""

    def __init__(self, navigation_state, task_registry, phase_provider, topology=None):
        self._navigation_state = navigation_state
        self._task_registry = task_registry
        self._phase_provider = phase_provider
        self._topology = topology

    def robot_state(self):
        return self._navigation_state.robot_state()

    def runtime_map_snapshot(self):
        return self._navigation_state.runtime_map_snapshot()

    def is_traversal_blocked(self, traversal_id):
        """按共享地图事实判断一条有向巡航边是否已被阻塞。"""

        if self._topology is None:
            raise RuntimeError("编排安全检查需要 TrackTopology")
        from_node_id, to_node_id = traversal_id.split("->", 1)
        cruise_edge = self._topology.get_cruise_edge(from_node_id, to_node_id)
        snapshot = self._navigation_state.runtime_map_snapshot()
        return any(snapshot.edge_status(edge_id).value == "blocked" for edge_id in cruise_edge.physical_edge_ids)

    def pending_tasks(self):
        return tuple(self._task_registry.pending_tasks()) if self._task_registry is not None else ()

    def mission_phase(self):
        state = self._phase_provider()
        if state is CoordinatorState.RETURNING:
            return PlanningPhase.RETURNING
        return PlanningPhase.TASK_PROCESSING

    def mission_finished(self):
        if self._task_registry is None:
            return False
        tasks = self._task_registry.list_tasks()
        return bool(tasks) and all(task.lifecycle is TaskLifecycle.COMPLETED for task in tasks)
