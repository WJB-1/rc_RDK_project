"""装配仿真世界、执行器和导航门面，并提供单步事件推进。"""

from typing import Callable, Optional
import time
import threading

from navigation.domain import build_default_topology
from .executor import SimExecutor
from .ports import SimMotionPort, SimPerceptionPort, SimTaskPort
from .snapshot import SimulationSnapshot
from .world import SimWorld


class SimulationRunner:
    """仿真事件循环门面；每次 step 最多完成一条异步请求。"""

    def __init__(self, world: Optional[SimWorld] = None, executor: Optional[SimExecutor] = None,
                 navigation_runtime=None, factory: Optional[Callable] = None, seed: int = 0) -> None:
        """创建一次仿真会话；复杂导航依赖可由 factory 负责装配。"""

        self._factory = factory
        self._seed = seed if world is None else world.seed
        self._world = world or SimWorld(build_default_topology(), seed=self._seed)
        self._executor = executor or SimExecutor((SimMotionPort(self._world), SimPerceptionPort(self._world), SimTaskPort(self._world)))
        self._navigation_runtime = navigation_runtime
        self._running = False
        self._paused = False
        self._stopped = False
        self._completed_action_count = 0
        self._last_outcome = None
        self._last_perception_frame = None
        self._timeline = []
        self._debug_mode = "manual"
        self._auto_stop = threading.Event()
        self._auto_thread = None
        self._step_lock = threading.Lock()

    @property
    def world(self):
        """返回仿真世界引用，业务状态仍只能通过快照观察。"""

        return self._world

    @property
    def executor(self):
        """返回装配的仿真执行器，供运行时或测试推进终局。"""

        return self._executor

    @property
    def navigation_runtime(self):
        """返回已装配的导航门面，供调试工具读取其公共快照。"""

        return self._navigation_runtime

    @property
    def state_store(self):
        """返回导航状态存储（若装配根提供），供仿真验收读取。"""

        return getattr(self._navigation_runtime, "state_store", None)

    def start(self) -> None:
        """启动会话并只调用一次 NavigationRuntime.start。"""

        if self._stopped or self._running:
            return
        self._running = True
        self._paused = False
        if self._navigation_runtime is not None:
            self._navigation_runtime.start()
        if self._debug_mode == "auto":
            self._start_auto_loop()

    def pause(self) -> None:
        """暂停异步事件推进，不改变世界或导航状态。"""

        if self._running and not self._stopped:
            self._paused = True

    def resume(self) -> None:
        """恢复被暂停的事件推进。"""

        if self._running and not self._stopped:
            self._paused = False

    def stop(self) -> None:
        """停止会话并阻止后续单步推进；重复调用幂等。"""

        self._stopped = True
        self._running = False
        self._paused = False
        self._auto_stop.set()

    def step(self) -> bool:
        """推进一个待完成端口事件，暂停、停止或无事件时返回 False。"""

        if not self._running or self._paused or self._stopped:
            return False
        with self._step_lock:
            if not self._executor.complete_next():
                return False
            self._completed_action_count += 1
            self._timeline.append((self._world.snapshot().now, self._completed_action_count))
            return True

    def step_once(self) -> bool:
        """Advance one simulated port event while preserving pause state."""

        was_paused = self._paused
        self._paused = False
        try:
            return self.step()
        finally:
            self._paused = was_paused

    @property
    def debug_mode(self) -> str:
        return self._debug_mode

    def set_mode(self, mode: str) -> str:
        selected = str(mode or "").lower()
        if selected not in ("manual", "auto"):
            raise ValueError("mode must be manual or auto")
        self._debug_mode = selected
        if selected == "auto" and self._running and not self._stopped:
            self._start_auto_loop()
        return selected

    def _start_auto_loop(self) -> None:
        if self._auto_thread is not None and self._auto_thread.is_alive():
            return
        self._auto_stop.clear()
        self._auto_thread = threading.Thread(target=self._auto_loop, daemon=True)
        self._auto_thread.start()

    def _auto_loop(self) -> None:
        while not self._auto_stop.wait(0.02):
            if self._debug_mode == "auto" and not self._paused:
                self.step()

    def run_until_observation(self, limit: int = 100) -> bool:
        """最多单步 limit 次，直到世界产生一帧观察结果。"""

        initial_count = self._world.snapshot().observation_count
        for _ in range(limit):
            if self._world.snapshot().observation_count > initial_count:
                return True
            if not self.step():
                break
        return self._world.snapshot().observation_count > initial_count

    def reset(self, seed: int = 0) -> None:
        """销毁当前会话并通过装配工厂创建全新世界和导航依赖。"""

        if self._factory is not None:
            replacement = self._factory(seed)
            self._world, self._executor, self._navigation_runtime = replacement
        else:
            self._world = SimWorld(build_default_topology(), seed=seed)
            self._executor = SimExecutor((SimMotionPort(self._world), SimPerceptionPort(self._world), SimTaskPort(self._world)))
            self._navigation_runtime = None
        self._seed = seed
        self._running = self._paused = self._stopped = False
        self._completed_action_count = 0
        self._last_outcome = self._last_perception_frame = None
        self._timeline = []

    def snapshot(self) -> SimulationSnapshot:
        """返回 Web 和调试面板唯一允许读取的组合快照。"""

        navigation_snapshot = (
            self._navigation_runtime.snapshot()
            if self._navigation_runtime is not None else None
        )
        world_snapshot = self._world.snapshot()

        robot_state_pose = None
        robot_location_label = None
        store = self.state_store
        if store is not None:
            rs = store.robot_state()
            if rs is not None:
                robot_state_pose = rs.world_pose
                loc = rs.location
                if hasattr(loc, "node_id"):
                    robot_location_label = "AtNode({})".format(loc.node_id)
                elif hasattr(loc, "traversal_id"):
                    robot_location_label = "Edge({}, {:.0f}mm)".format(
                        loc.traversal_id, getattr(loc, "progress_mm", 0.0)
                    )

        return SimulationSnapshot(
            self._running, self._paused, self._stopped, self._seed,
            world_snapshot.now, world_snapshot,
            self._completed_action_count, None, self._last_outcome,
            self._last_perception_frame, navigation_snapshot,
            tuple(self._timeline),
            robot_state_pose=robot_state_pose,
            robot_location_label=robot_location_label,
        )


class HardwareMotionSimulationRunner(SimulationRunner):
    """保持仿真感知与任务，但把运动请求交给真实 STM32 的会话。"""

    def __init__(self, motion_port, transport=None, vision_runtime=None,
                 ready_timeout_s=2.0, **kwargs) -> None:
        super().__init__(**kwargs)
        self.motion_port = motion_port
        self.transport = transport
        self.vision_runtime = vision_runtime
        self._ready_timeout_s = float(ready_timeout_s)

    def start(self) -> None:
        if self._stopped or self._running:
            return
        if self.vision_runtime is not None:
            self.vision_runtime.start()
        self.motion_port.start()
        deadline = time.monotonic() + self._ready_timeout_s
        while not self.motion_port.is_ready and time.monotonic() < deadline:
            time.sleep(0.01)
        if not self.motion_port.is_ready:
            self.motion_port.stop()
            if self.vision_runtime is not None:
                self.vision_runtime.stop()
            raise TimeoutError("STM32 motion link did not acknowledge HELLO")
        super().start()

    def reset(self, seed: int = 0) -> None:
        raise RuntimeError("hardware motion session cannot be reset; restart the process")

    def stop(self) -> None:
        super().stop()
        self.motion_port.stop()
        if self.vision_runtime is not None:
            self.vision_runtime.stop()
