import unittest
import time

from navigation.contracts import ExecutionRequest, ExecutionTarget, DriveExecutionCommand
from navigation.domain import AtNode, build_default_topology
from navigation.simulation import SimExecutor, SimMotionPort, SimWorld, SimulationRunner


class FakeRuntime:
    def __init__(self):
        self.started = 0

    def start(self):
        self.started += 1

    def snapshot(self):
        return {"started": self.started}


class SimulationRunnerTest(unittest.TestCase):
    def test_composed_navigation_registers_check_in_tasks_without_culvert_quota(self):
        from navigation.simulation.composition import build_simulation_runner

        runner = build_simulation_runner(seed=3)
        snapshot = runner.snapshot().navigation

        self.assertEqual(len(snapshot.tasks), 11)
        self.assertEqual(snapshot.task_progress["check_in_total"], 11)
        self.assertIn("check-in-N1", {task.task_id for task in snapshot.tasks})
        self.assertEqual(snapshot.task_progress["culvert_target"], 0)

    def test_task_window_handoff_allows_replanning_from_target_node(self):
        from navigation.simulation.composition import build_simulation_runner

        runner = build_simulation_runner(seed=0)
        runner.start()
        for _ in range(18):
            self.assertTrue(runner.step())

        location = runner.state_store.robot_state().location
        self.assertIsInstance(location, AtNode)
        self.assertEqual(location.node_id, "N2")
        self.assertIsNotNone(runner.navigation_runtime.snapshot().active_choreography)
    def test_runner_step_completes_one_navigation_action(self):
        world = SimWorld(build_default_topology(), seed=3)
        port = SimMotionPort(world)
        executor = SimExecutor((port,))
        executor.on_interrupt(lambda interrupt: None)
        executor.submit(ExecutionRequest("r", "e", "a", ExecutionTarget.MOTION_CONTROLLER,
                                         DriveExecutionCommand(100), 0.0))
        runner = SimulationRunner(world, executor, FakeRuntime())
        runner.start()
        self.assertTrue(runner.step())
        self.assertEqual(runner.snapshot().completed_action_count, 1)
        self.assertFalse(runner.step())

    def test_paused_runner_does_not_advance(self):
        world = SimWorld(build_default_topology(), seed=3)
        port = SimMotionPort(world)
        executor = SimExecutor((port,))
        executor.on_interrupt(lambda interrupt: None)
        executor.submit(ExecutionRequest("r", "e", "a", ExecutionTarget.MOTION_CONTROLLER,
                                         DriveExecutionCommand(100), 0.0))
        runner = SimulationRunner(world, executor, FakeRuntime())
        runner.start()
        runner.pause()
        self.assertFalse(runner.step())
        self.assertEqual(runner.snapshot().completed_action_count, 0)

    def test_auto_mode_completes_pending_simulated_event(self):
        world = SimWorld(build_default_topology(), seed=3)
        port = SimMotionPort(world)
        executor = SimExecutor((port,))
        executor.on_interrupt(lambda interrupt: None)
        executor.submit(ExecutionRequest("r", "e", "a", ExecutionTarget.MOTION_CONTROLLER,
                                         DriveExecutionCommand(100), 0.0))
        runner = SimulationRunner(world, executor, FakeRuntime())
        runner.set_mode("auto")
        runner.start()
        deadline = time.monotonic() + 0.5
        while runner.snapshot().completed_action_count == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        runner.stop()

        self.assertEqual(runner.debug_mode, "auto")
        self.assertEqual(runner.snapshot().completed_action_count, 1)


if __name__ == "__main__":
    unittest.main()
