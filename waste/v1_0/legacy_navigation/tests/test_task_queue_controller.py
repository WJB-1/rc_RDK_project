"""
test_task_queue_controller.py — 任务队列控制器的摸底单测

目的：验证「任务队列驱动」能否纯函数式表达原状态机的四类语义，
重点摸清「倒车动画」能否脱离 agent 物理字段（_backtrack_distance）。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import unittest
from navigation.control.task_queue_controller import Task, TaskQueueController


class TestTaskQueueController(unittest.TestCase):

    def setUp(self):
        self.c = TaskQueueController()

    # ----------------------------------------------------------------
    # 1. 障碍中断 → 队列作废 + 注入 reverse 指令
    # ----------------------------------------------------------------
    def test_obstacle_invalidates_and_injects_reverse(self):
        # 先塞一段正常前进队列
        self.c.queue = [
            Task(kind="drive", trigger="distance_reached", params={"distance_mm": 800}),
            Task(kind="turn", trigger="angle_reached", params={"heading": 90}),
        ]
        self.c.cursor = 0

        # 障碍事件 → 作废 + 倒车
        self.c.on_obstacle(blocked_edge_id=5, reverse_distance_mm=640.0,
                           reverse_heading=180.0)

        self.assertEqual(len(self.c.queue), 1)
        self.assertEqual(self.c.queue[0].kind, "reverse")
        self.assertEqual(self.c.queue[0].params["distance_mm"], 640.0)

    # ----------------------------------------------------------------
    # 2. 倒车到位（距离达到）→ 推进，而非跨 tick 累积
    # ----------------------------------------------------------------
    def test_reverse_done_by_distance_not_tick_accumulation(self):
        self.c.queue = [
            Task(kind="reverse", trigger="distance_reached",
                 params={"distance_mm": 640.0}),
            Task(kind="drive", trigger="distance_reached", params={"distance_mm": 800}),
        ]
        self.c.cursor = 0

        # 还没退够
        self.c.on_reverse_done(reached_distance_mm=300.0)
        self.assertEqual(self.c.cursor, 0)   # 还没推进

        # 退够了 → 推进到下一个
        self.c.on_reverse_done(reached_distance_mm=640.0)
        self.assertEqual(self.c.cursor, 1)
        self.assertEqual(self.c.peek().kind, "drive")

    # ----------------------------------------------------------------
    # 3. 涵洞 → 注入探索子程序，而非切状态
    # ----------------------------------------------------------------
    def test_culvert_injects_probe_program(self):
        self.c.queue = [Task(kind="drive", trigger="distance_reached",
                             params={"distance_mm": 800})]
        self.c.cursor = 0

        self.c.on_culvert(edge_id=73)

        self.assertEqual(len(self.c.queue), 2)
        self.assertEqual(self.c.queue[1].kind, "culvert_probe")
        self.assertEqual(self.c.queue[1].params["edge_id"], 73)

    # ----------------------------------------------------------------
    # 4. 到达终点 → 队列空返回「待规划」而非「完成」
    # ----------------------------------------------------------------
    def test_exhausted_means_replan_not_finished(self):
        self.c.queue = [Task(kind="drive", trigger="distance_reached",
                             params={"distance_mm": 100})]
        self.c.cursor = 0

        self.c.advance()   # 走完唯一任务
        self.assertTrue(self.c.is_exhausted())   # 队列空 = 待规划，不是"完成"

    # ----------------------------------------------------------------
    # 5. 到达打卡点 → 注入 checkpoint 任务
    # ----------------------------------------------------------------
    def test_arrive_checkpoint_injects_checkpoint(self):
        self.c.queue = [Task(kind="drive", trigger="distance_reached",
                             params={"distance_mm": 800})]
        self.c.cursor = 0

        self.c.on_arrive(node="N1", has_checkpoint=True)
        self.assertEqual(self.c.queue[-1].kind, "checkpoint")
        self.assertEqual(self.c.queue[-1].params["node"], "N1")


if __name__ == "__main__":
    unittest.main()
