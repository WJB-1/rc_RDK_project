"""仿真导航系统的统一组合根。"""

from navigation.choreography import Choreographer, MotionProfile
from navigation.coordinator import Coordinator
from navigation.coordinator.planning_state_adapter import CoordinatorPlanningReadAdapter
from navigation.domain import AtNode, NavigationStateStore, RobotState, RuntimeMap, Task, TaskKind, TaskRegistry
from navigation.perception_adapter import PerceptionAdapter
from navigation.planning import RecoveryPlanner, RoutePlanner
from navigation.runtime import NavigationRuntime

from .executor import SimExecutor
from .ports import SimMotionPort, SimPerceptionPort, SimTaskPort
from .runner import SimulationRunner
from .world import SimWorld


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

    world = SimWorld(seed=seed)
    topology = world.topology
    runtime_map = RuntimeMap()
    state_store = NavigationStateStore(runtime_map, RobotState(AtNode("START"), 90.0))
    task_registry = TaskRegistry((Task("check-in-n12", TaskKind.CHECK_IN, "N12"),))
    coordinator_holder = {}
    state_query = CoordinatorPlanningReadAdapter(
        state_store,
        task_registry,
        lambda: coordinator_holder["coordinator"].state,
        topology,
    )
    choreographer = Choreographer(
        topology,
        MotionProfile(initial_observation_advance_mm=300.0),
        state_query,
        task_registry,
    )
    route_planner = RoutePlanner(topology, state_query)
    recovery_planner = RecoveryPlanner(topology)
    perception_adapter = PerceptionAdapter(state_store, topology)
    executor = SimExecutor((SimMotionPort(world), SimPerceptionPort(world), SimTaskPort(world)))
    coordinator = Coordinator(
        executor=executor,
        choreographer=choreographer,
        navigation_state=state_store,
        perception_adapter=perception_adapter,
        topology=topology,
        task_registry=task_registry,
        route_planner=route_planner,
        recovery_planner=recovery_planner,
    )
    coordinator_holder["coordinator"] = coordinator
    return world, executor, NavigationRuntime(coordinator)
