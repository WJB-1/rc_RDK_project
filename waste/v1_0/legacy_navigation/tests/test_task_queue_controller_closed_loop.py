"""
test_task_queue_controller_closed_loop.py — 边级闭环单测

验证四类任务（drive/turn/reverse/culvert_probe）都能 step + trigger 推进，
且保持纯函数（里程/进度作为显式参数传入，不 import agent）。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from dataclasses import dataclass, field

from navigation.control.task_queue_controller import Task, TaskQueueController
from navigation.contracts import TurnAction, TurnCommand


@dataclass
class FakeProgress:
    """EdgeProgress 的鸭子类型替身（纯函数单测用，不 import EdgeExecutor）。"""
    progress_ratio: float = 0.0
    timeout: bool = False
    is_stalled: bool = False
    distance_mm: float = 0.0


class TestDriveClosedLoop(unittest.TestCase):

    def setUp(self):
        self.c = TaskQueueController()

    def _setup_drive(self):
        self.c.queue = [
            Task(kind="drive", trigger="distance_reached", params={"distance_mm": 800}),
            Task(kind="turn", trigger="angle_reached",
                 params={"target_node": "N2", "expected_yaw": 90.0, "current_yaw": 0.0}),
        ]
        self.c.cursor = 0

    def test_drive_advances_on_window_ratio(self):
        self._setup_drive()
        # 达到检测窗口比例（0.95）→ 推进到 turn
        self.c.step(FakeProgress(progress_ratio=0.96))
        self.assertEqual(self.c.cursor, 1)
        self.assertEqual(self.c.peek().kind, "turn")

    def test_drive_not_advance_before_window(self):
        self._setup_drive()
        self.c.step(FakeProgress(progress_ratio=0.5))
        self.assertEqual(self.c.cursor, 0)   # 还不够

    def test_drive_advances_on_timeout(self):
        self._setup_drive()
        self.c.step(FakeProgress(progress_ratio=0.2, timeout=True))
        self.assertEqual(self.c.cursor, 1)

    def test_drive_advances_on_stalled_at_complete(self):
        self._setup_drive()
        self.c.step(FakeProgress(progress_ratio=1.0, is_stalled=True))
        self.assertEqual(self.c.cursor, 1)


class TestTurnClosedLoop(unittest.TestCase):

    def setUp(self):
        self.c = TaskQueueController()
        self.c.queue = [
            Task(kind="turn", trigger="angle_reached",
                 params={"target_node": "N2", "expected_yaw": 90.0, "current_yaw": 0.0}),
            Task(kind="drive", trigger="distance_reached", params={"distance_mm": 800}),
        ]
        self.c.cursor = 0

    def test_turn_produces_command(self):
        cmd = self.c.step(FakeProgress())
        self.assertIsInstance(cmd, TurnCommand)
        self.assertEqual(cmd.target_node, "N2")
        self.assertEqual(cmd.expected_yaw, 90.0)
        self.assertEqual(cmd.action, TurnAction.TURN_LEFT)   # 0 → 90 = 左转

    def test_turn_advances_on_done(self):
        self.c.on_turn_done()
        self.assertEqual(self.c.cursor, 1)
        self.assertEqual(self.c.peek().kind, "drive")

    def test_turn_done_noop_if_not_turn(self):
        self.c.cursor = 1   # 队首是 drive
        self.c.on_turn_done()
        self.assertEqual(self.c.cursor, 1)   # 不推进


class TestReverseClosedLoop(unittest.TestCase):

    def setUp(self):
        self.c = TaskQueueController()
        self.c.queue = [
            Task(kind="reverse", trigger="distance_reached", params={"distance_mm": 640.0}),
            Task(kind="drive", trigger="distance_reached", params={"distance_mm": 800}),
        ]
        self.c.cursor = 0

    def test_reverse_advances_when_distance_reached(self):
        self.c.step(FakeProgress(distance_mm=640.0))
        self.assertEqual(self.c.cursor, 1)
        self.assertEqual(self.c.peek().kind, "drive")

    def test_reverse_not_advance_before_distance(self):
        self.c.step(FakeProgress(distance_mm=300.0))
        self.assertEqual(self.c.cursor, 0)


class TestCulvertProbeClosedLoop(unittest.TestCase):

    def setUp(self):
        self.c = TaskQueueController()
        self.c.queue = [
            Task(kind="culvert_probe", trigger="data_acked", params={"edge_id": 73}),
            Task(kind="drive", trigger="distance_reached", params={"distance_mm": 800}),
        ]
        self.c.cursor = 0

    def test_culvert_advances_on_data_acked(self):
        self.c.on_data_acked()
        self.assertEqual(self.c.cursor, 1)
        self.assertEqual(self.c.peek().kind, "drive")

    def test_culvert_not_advance_on_tick(self):
        # culvert_probe 不靠 tick 推进
        self.c.step(FakeProgress(progress_ratio=1.0))
        self.assertEqual(self.c.cursor, 0)


if __name__ == "__main__":
    unittest.main()
