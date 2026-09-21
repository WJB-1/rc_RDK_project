"""仿真导航系统的统一组合根。"""

from navigation.choreography.choreographer import Choreographer
from navigation.coordinator import Coordinator
from navigation.coordinator.planning_state_adapter import CoordinatorPlanningReadAdapter
from navigation.domain import AtNode, NavigationStateStore, RobotState, RuntimeMap, Task, TaskKind, TaskRegistry, build_default_topology
from navigation.perception_adapter import PerceptionAdapter
from navigation.planning import RecoveryPlanner, RoutePlanner
from navigation.runtime import NavigationRuntime

from .executor import SimExecutor
from .ports import SimMotionPort, SimPerceptionPort, SimTaskPort
from .runner import SimulationRunner
from .world import SimWorld
from navigation.domain.state import WorldPose


def build_simulation_runner(seed: int = 0) -> SimulationRunner:
    """创建包含 Coordinator 的可运行仿真会话。"""

    world, executor, runtime = _compose(seed)
    return SimulationRunner(
        world=world,
        executor=executor,
        navigation_runtime=runtime,
        factory=lambda factory_seed: _compose(factory_seed),
        seed=seed,
    )


def _compose(seed: int):
    """装配一套彼此隔离的世界、执行器和导航依赖。"""

    topology = build_default_topology()
    start_node = topology.get_node("START")
    runtime_map = RuntimeMap()
    state_store = NavigationStateStore(
        runtime_map,
        RobotState(
            AtNode("START"),
            WorldPose(start_node.x_mm, start_node.y_mm, 90.0),
        ),
    )
    # 世界位姿由 NavigationState 唯一维护；SimWorld 只读 RobotState.world_pose。
    world = SimWorld(
        topology=topology,
        seed=seed,
        initial_pose=WorldPose(start_node.x_mm, start_node.y_mm, 90.0),
        pose_provider=lambda: state_store.robot_state().world_pose,
    )
    topology = world.topology
    task_registry = TaskRegistry(tuple(
        Task("check-in-N{}".format(node_id), TaskKind.CHECK_IN, "N{}".format(node_id))
        for node_id in range(1, 12)
    ))
    route_planner = RoutePlanner(topology, None)
    recovery_planner = RecoveryPlanner(topology)
    perception_adapter = PerceptionAdapter(state_store, topology)
    executor = SimExecutor((SimMotionPort(world), SimPerceptionPort(world), SimTaskPort(world)))
    coordinator = Coordinator(
        executor=executor,
        navigation_state=state_store,
        perception_adapter=perception_adapter,
        topology=topology,
        task_registry=task_registry,
        route_planner=route_planner,
        recovery_planner=recovery_planner,
        culvert_quota=8,
    )
    return world, executor, NavigationRuntime(coordinator)