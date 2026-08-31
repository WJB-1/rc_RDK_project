"""
control/task_queue_controller.py — 任务队列控制器（纯函数 边级闭环版）

这是「任务队列驱动、废除状态机」架构的核心，显式化状态机背后的语义为
一个纯函数队列控制器。四类边级任务都能 step + trigger 推进：

  drive        —— 里程推进（executor.update 的 progress_ratio/timeout/stalled）
  turn         —— 产出 TurnCommand，靠 on_turn_done 回执推进
  reverse      —— 里程回退（单条 reverse 的 distance_reached，非跨 tick 累积）
  culvert_probe —— 走单点写入，靠 on_data_acked 回执推进

纯函数式：同样的 (queue, cursor, 里程/进度, 事件) 输入，产出同样结果。无副作用。
不 import agent / sim_engine / runtime_map。可独立单测。

关键设计（为何里程/进度作为显式参数传入）：
  drive/reverse 的推进依赖「当前里程」，而纯函数控制器不能持有 agent 引用。
  因此 step() 从外部接收进度快照（EdgeProgress 的鸭子类型），驱动所需的
  里程随参数流入而非由控制器持有，保持纯函数、可独立单测。
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

try:
    from ..contracts import TurnAction, TurnCommand, PlanningInfeasibleError
except ImportError:  # 独立单测直接运行时无相对包上下文
    from navigation.contracts import TurnAction, TurnCommand, PlanningInfeasibleError


@dataclass
class Task:
    """队列单元：统一类型，用 kind 区分动作/感知/任务程序。"""
    kind: str            # "drive" | "turn" | "reverse" | "checkpoint" | "culvert_probe"
    trigger: str         # 完成条件："distance_reached" | "angle_reached" | "data_acked"
    params: Dict = field(default_factory=dict)   # 距离、航向、目标点等


class TaskQueueController:
    """
    纯函数式队列控制器。

    输入：队列 + 游标 + 里程/进度快照（step）+ 事件（trigger）
    输出：修改自身 queue/cursor（但所有逻辑是确定性的纯变换）

    闭环目标：四类边级任务都能 step + trigger 推进，脱离 agent 物理字段。
    """

    def __init__(self):
        self.queue: List[Task] = []
        self.cursor: int = 0

    def load_action_plan(self, action_plan) -> None:
        """Compatibility adapter from the new immutable action plan."""
        self.queue = [Task(kind=command.kind, trigger="feedback",
                           params=dict(command.parameters))
                      for command in action_plan.actions]
        self.cursor = 0

    def cancel(self) -> None:
        self.queue = []
        self.cursor = 0

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

        死代码（Task 28 第 4 步收尸标注）：门面换核后从未调用本方法，checkpoint
        任务不靠此路径注入。待 D-16「涵洞发起/验收」落实后启用。勿引入新调用方
        扩大范围。
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

    # ================================================================
    # 边级闭环：step + trigger
    # ================================================================

    def step(self, progress, now: Optional[float] = None) -> Optional[TurnCommand]:
        """
        每 tick 驱动队首任务，按 kind 分派推进。

        里程/进度作为显式参数传入（progress 为 EdgeProgress 的鸭子类型），
        控制器不持有 agent 引用，保持纯函数。

        :param progress: EdgeProgress 快照（drive 用 progress_ratio/timeout/is_stalled；
                         reverse 用 distance_mm 作为回退距离）
        :return: turn 类任务产出 TurnCommand；其余返回 None。
        """
        t = self.peek()
        if t is None or self.is_exhausted():
            return None

        if t.kind == "drive":
            self._step_drive(t, progress)
            return None

        if t.kind == "turn":
            # turn 的推进不靠 tick，靠 on_turn_done 回执；但每 tick 都产出指令
            # 供下位机执行（幂等，直到回执推进游标）。
            return self.produce_turn_command()

        if t.kind == "reverse":
            self._step_reverse(t, progress)
            return None

        # culvert_probe / checkpoint 不靠 tick 推进，靠 on_data_acked
        return None

    def produce_turn_command(self) -> Optional[TurnCommand]:
        """
        turn 类任务 → 产出 TurnCommand(action, target_node, expected_yaw)。

        纯函数式 _determine_turn：由当前 yaw（params.current_yaw）与 target 期望
        yaw（params.expected_yaw）比较得到 TurnAction，防御 180° 掉头 → STOP。
        """
        t = self.peek()
        if t is None or t.kind != "turn":
            return None

        target_node = t.params.get("target_node", "")
        expected_yaw = float(t.params.get("expected_yaw", 0.0))
        current_yaw = float(t.params.get("current_yaw", 0.0))

        action = self._determine_turn(current_yaw, expected_yaw)
        return TurnCommand(action=action, target_node=target_node,
                           expected_yaw=expected_yaw)

    @staticmethod
    def _determine_turn(current_yaw: float, expected_yaw: float) -> TurnAction:
        """
        与旧内核 _determine_turn 语义等价（阈值仍走缺省，可被上层覆写）。
        180° 掉头 = 硬逻辑错误（上层漏传朝向），fail fast 抛错，绝不降级 STOP。
        """
        import config as _cfg

        diff = expected_yaw - current_yaw
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        if abs(diff) < _cfg.get("state_machine.turn_straight_deg", 15.0):
            return TurnAction.STRAIGHT
        if abs(diff) > _cfg.get("state_machine.turn_uturn_deg", 160.0):
            raise PlanningInfeasibleError(
                f"非法 180° 掉头 diff={diff:.1f} → 兜底筛查失败，当前 yaw={current_yaw:.1f} "
                f"期望 yaw={expected_yaw:.1f}"
            )
        return TurnAction.TURN_LEFT if diff > 0 else TurnAction.TURN_RIGHT

    def on_turn_done(self):
        """
        turn 回执：下位机确认转弯完成 → 推进游标。
        与旧 _on_approach_done + on_turn_done + _on_node_arrival 的「转向结束」等价。
        """
        t = self.peek()
        if t is not None and t.kind == "turn":
            self.advance()

    def on_data_acked(self):
        """
        culvert_probe / checkpoint 回执：单点写入完成（data_acked）→ 推进。
        与旧 _handle_culvert_recon 的「侦查完成」等价，触发推进而非切状态。

        死代码标注（Task 28 第 4 步收尸）：checkpoint 任务的推进路径当前未被门面
        使用（门面换核后只有 culvert_probe 走此方法），待 D-16 落实后启用 checkpoint。
        """
        t = self.peek()
        if t is not None and t.trigger == "data_acked":
            self.advance()

    # ----------------------------------------------------------------
    # 各 kind 的 tick 推进
    # ----------------------------------------------------------------

    def _step_drive(self, task: Task, progress) -> None:
        """
        drive 里程推进：跟进旧 _tick_edge_executing。

        达到检测窗口比例 / 超时 / 完成且停滞 → 推进（等价于旧「强行到达」）。
        """
        import config as _cfg

        ratio = getattr(progress, "progress_ratio", 1.0)
        timeout = getattr(progress, "timeout", False)
        is_stalled = getattr(progress, "is_stalled", False)

        window_ratio = _cfg.get("state_machine.crossroad_detection_window_ratio", 0.95)
        if ratio >= window_ratio or timeout or (ratio >= 1.0 and is_stalled):
            self.advance()

    def _step_reverse(self, task: Task, progress) -> None:
        """
        reverse 里程回退：用 distance_reached 触发（非跨 tick 累积）。

        progress.distance_mm 是「已回退距离」，达到 params.distance_mm → 推进。
        """
        required = task.params.get("distance_mm", 0.0)
        reached = getattr(progress, "distance_mm", 0.0)
        if reached >= required:
            self.advance()
