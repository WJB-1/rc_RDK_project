"""
control/task_queue_controller.py — 任务队列控制器（纯函数摸底版）

这是「任务队列驱动、废除状态机」架构的第一步摸底：
把状态机背后的「语义」显式化为一个纯函数队列控制器，验证——
「倒车动画」「障碍中断」「涵洞注入」「终点判断」这四类状态语义，
能否脱离 agent 的物理字段（yaw_deg/_backtrack_distance）纯粹表达。

纯函数式：同样的 (queue, cursor, 事件) 输入，产出同样结果。无副作用。
不 import agent / sim_engine。可独立单测。

关键试验点：倒车用 Task(kind="reverse", trigger="distance_reached", params={distance_mm})
表达，而非跨 tick 累积 _backtrack_distance。本文件要证明这样是否可行。
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class Task:
    """队列单元：统一类型，用 kind 区分动作/感知/任务程序。"""
    kind: str            # "drive" | "turn" | "reverse" | "checkpoint" | "culvert_probe"
    trigger: str         # 完成条件："distance_reached" | "angle_reached" | "data_acked"
    params: Dict = field(default_factory=dict)   # 距离、航向、目标点等


class TaskQueueController:
    """
    纯函数式队列控制器。

    输入：队列 + 游标 + 一个事件
    输出：修改自身 queue/cursor（但所有逻辑是确定性的纯变换）

    摸底目标：验证四类状态语义能否纯表达。不碰 agent 物理字段。
    """

    def __init__(self):
        self.queue: List[Task] = []
        self.cursor: int = 0

    # ================================================================
    # 游标操作
    # ================================================================

    def peek(self) -> Optional[Task]:
        """取游标当前任务。"""
        if 0 <= self.cursor < len(self.queue):
            return self.queue[self.cursor]
        return None

    def advance(self):
        """推进游标（当前任务完成）。"""
        self.cursor += 1

    def is_exhausted(self) -> bool:
        """队列是否走空（= 触发"待规划"而非"完成"）。"""
        return self.cursor >= len(self.queue)

    # ================================================================
    # 事件驱动（四类语义）
    # ================================================================

    def on_obstacle(self, blocked_edge_id: int, reverse_distance_mm: float,
                    reverse_heading: float):
        """
        障碍中断语义：作废当前队列 + 注入倒车脱困指令。

        关键试验：倒车用 reverse 指令的 distance_mm 触发（非跨 tick 累积）。
        """
        # 作废当前队列，替换成倒车脱困序列
        self.queue = [
            Task(kind="reverse", trigger="distance_reached",
                 params={"distance_mm": reverse_distance_mm,
                         "heading": reverse_heading}),
        ]
        self.cursor = 0
        # 障碍边 blocked 的记录交给上层（本纯函数不碰 runtime_map）

    def on_reverse_done(self, reached_distance_mm: float):
        """
        倒车到位语义：距离达到 → 推进。
        对照旧「跨 tick 累积 _backtrack_distance」——这里用单条 reverse 的
        distance_reached 触发即可推进，无需跨 tick。
        """
        t = self.peek()
        if t is None or t.kind != "reverse":
            return
        required = t.params.get("distance_mm", 0.0)
        if reached_distance_mm >= required:
            self.advance()

    def on_culvert(self, edge_id: int):
        """
        涵洞注入语义：在队列中注入涵洞探索子程序，而非切状态。
        """
        self.queue.append(
            Task(kind="culvert_probe", trigger="data_acked",
                 params={"edge_id": edge_id})
        )

    def on_arrive(self, node: str, has_checkpoint: bool):
        """
        到达终点语义：若节点是打卡点，注入打卡任务；否则推进。
        队列走空时返回「待规划」而非「完成」。
        """
        if has_checkpoint:
            self.queue.append(
                Task(kind="checkpoint", trigger="data_acked",
                     params={"node": node})
            )
        # 到达不在队列里注入额外的"完成"标志——由 is_exhausted 判定

    def invalidate_and_replace(self, new_tasks: List[Task]):
        """作废当前队列 + 替换为新规划（重规划语义）。"""
        self.queue = list(new_tasks)
        self.cursor = 0
