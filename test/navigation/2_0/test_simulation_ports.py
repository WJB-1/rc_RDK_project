import unittest
from types import SimpleNamespace

from navigation.contracts import (
    DispatchAck, ExecutionOutcome, ExecutionRequest, ExecutionTarget,
    DriveExecutionCommand, ExecuteTaskExecutionCommand, ObserveExecutionCommand, PerceptionFrame,
)
from navigation.domain import build_default_topology
from navigation.simulation.ports import SimMotionPort, SimPerceptionPort, SimTaskPort
from navigation.simulation.world import SimWorld


def request(target, command, name="a"):
    return ExecutionRequest(name, "e", name, target, command, 0.0)


class SimulationPortsTest(unittest.TestCase):
    def test_motion_port_reports_odometry_only_through_completion(self):
        port = SimMotionPort(SimWorld(build_default_topology(), seed=1))
        events = []
        ack = port.submit(request(ExecutionTarget.MOTION_CONTROLLER, DriveExecutionCommand(100)), events.append)
        self.assertTrue(ack.accepted)
        completion = port.complete_next()
        self.assertEqual(completion.outcome, ExecutionOutcome.COMPLETED)
        self.assertGreater(completion.odometry_delta_mm, 0)

    def test_perception_port_returns_frame_on_observe_command(self):
        port = SimPerceptionPort(SimWorld(build_default_topology(), seed=1))
        port.submit(request(ExecutionTarget.PERCEPTION_SYSTEM, ObserveExecutionCommand()), lambda _: None)
        completion = port.complete_next()
        self.assertIsInstance(completion.perception_frame, PerceptionFrame)

    def test_task_port_explores_then_drives_visual_turn_window_distance(self):
        class RecordingWorld:
            def __init__(self):
                self.drive_distances = []
                self.executed_tasks = []

            def execute_task_drive(self, distance_mm):
                self.drive_distances.append(distance_mm)
                return SimpleNamespace(outcome=ExecutionOutcome.COMPLETED, timestamp=float(len(self.drive_distances)))

            def estimate_distance_to_next_turn_window_mm(self):
                return 180.0

            def execute_task(self, command):
                self.executed_tasks.append(command.task_id)
                return SimpleNamespace(outcome=ExecutionOutcome.COMPLETED, timestamp=3.0, task_result="completed", error_code=None)

            def snapshot(self):
                return SimpleNamespace(now=0.0)

        world = RecordingWorld()
        port = SimTaskPort(world)
        events = []
        port.submit(request(ExecutionTarget.TASK_SYSTEM, ExecuteTaskExecutionCommand("culvert-001")), events.append)

        completion = port.complete_next()

        self.assertEqual(completion.outcome, ExecutionOutcome.COMPLETED)
        self.assertEqual(world.drive_distances, [300.0, 180.0])
        self.assertEqual(world.executed_tasks, ["culvert-001"])

    def test_task_port_rejects_invalid_visual_turn_window_distance(self):
        class RecordingWorld:
            def __init__(self):
                self.drive_distances = []
                self.executed_task = False

            def execute_task_drive(self, distance_mm):
                self.drive_distances.append(distance_mm)
                return SimpleNamespace(outcome=ExecutionOutcome.COMPLETED, timestamp=1.0)

            def estimate_distance_to_next_turn_window_mm(self):
                return None

            def execute_task(self, command):
                self.executed_task = True
                raise AssertionError("invalid estimate must not execute task")

            def snapshot(self):
                return SimpleNamespace(now=0.0)

        world = RecordingWorld()
        port = SimTaskPort(world)
        port.submit(request(ExecutionTarget.TASK_SYSTEM, ExecuteTaskExecutionCommand("culvert-001")), lambda _: None)

        completion = port.complete_next()

        self.assertEqual(completion.outcome, ExecutionOutcome.FAILED)
        self.assertEqual(completion.error_code, "TURN_WINDOW_DISTANCE_UNAVAILABLE")
        self.assertEqual(world.drive_distances, [300.0])
        self.assertFalse(world.executed_task)

    def test_task_port_simulates_check_in_without_culvert_exploration(self):
        class RecordingWorld:
            def __init__(self):
                self.drive_distances = []
                self.executed_tasks = []

            def execute_task_drive(self, distance_mm):
                self.drive_distances.append(distance_mm)
                raise AssertionError("check-in must not explore a culvert")

            def execute_task(self, command):
                self.executed_tasks.append(command.task_id)
                return SimpleNamespace(outcome=ExecutionOutcome.COMPLETED, timestamp=1.0,
                                       task_result="checked-in", error_code=None)

            def snapshot(self):
                return SimpleNamespace(now=0.0)

        world = RecordingWorld()
        port = SimTaskPort(world)
        port.submit(request(ExecutionTarget.TASK_SYSTEM,
                            ExecuteTaskExecutionCommand("check-in-N1")), lambda _: None)

        completion = port.complete_next()

        self.assertEqual(completion.outcome, ExecutionOutcome.COMPLETED)
        self.assertEqual(completion.task_result, "checked-in")
        self.assertEqual(world.executed_tasks, ["check-in-N1"])


if __name__ == "__main__":
    unittest.main()
