"""
navigation/state_machine.py — Agent 核心控制 (V4.0 · 任务队列驱动)

Task 28 第 3 步「原子换核」：门面内核从 CruiseStateMachine（状态机）切成
TaskQueueController（任务队列）。`on_xxx` 语义重定义为「入队原语 + 副作用」。

`self.state` 退化为「队列→状态」的单向只读投影：只在入队原语 / 推进回调处
作为副作用写出（写一次随队列变化），**永远不得**成为内部逻辑的分支依据。
内部所有行为分支只看 `queue_controller.peek().kind`（队首任务类型）。
"""

import math
import time
from typing import Dict, List, Tuple, Optional

from .contracts import (
    AgentState, TurnAction, TurnCommand, Pose, NavigationState,
    OdomUpdate, RoadCondition, CrossroadEvent, CulvertEvent,
    ObstacleEvent, RfidEvent, EdgeTaskStatus,
)
from .domain.topology import RaceTrackTopology, get_topology
from .domain.runtime_map import RuntimeMap
from .planning.map_oracle import MapOracle
from .domain.config import NODE_COORDS
from .planning.path_planner import PathPlanner
from .control.edge_executor import EdgeExecutor
from .planning.deadend_recovery import DeadEndRecovery
from .planning.junction_decider import JunctionDecider
from .planning.task_queue_adapter import TaskQueueAdapter
from .control.orchestrator import StateOrchestrator
from .control.task_queue_controller import TaskQueueController, Task
try:
    from .. import config as _cfg
except ImportError:
    import config as _cfg


class AgentStateMachine:
    """
    Agent 导航大脑 (V4.0 · 任务队列驱动)

    职责:
    1. 维护车辆物理坐标与拓扑定位
    2. 管理边级任务执行（队列驱动，队首 kind 分派）
    3. 响应感知事件，入队任务 / 推进游标
    4. 调度 vision 工具进行涵洞/障碍物检测
    """

    def __init__(self, topology: RaceTrackTopology = None):
        self.topo = topology or get_topology()
        self.oracle = MapOracle(self.topo)
        self.planner = PathPlanner(self.oracle, self.topo)
        self.executor = EdgeExecutor()

        # 时钟（统一计时）：默认真实墙钟；仿真注入 sim_time（见 set_clock）
        self._clock = time.time

        # 门面硬拆：职责对象（做法 A）
        self._recovery = DeadEndRecovery(self)      # 死胡同倒车恢复
        self._orchestrator = StateOrchestrator(self)  # 宏观调度

        # Task 28 第 3 步：内核换成任务队列控制器（queue_controller，避开 _task_queue adapter）
        self.queue_controller = TaskQueueController()

        # 路口决策器（D-03）+ 任务队列适配（只读观察，给 JunctionDecider 暴露
        # pending_rfid/pending_culverts；本 adapter 不承担执行队列）
        self._task_queue = TaskQueueAdapter(self)
        self._junction_decider = JunctionDecider(self.oracle, self.topo)

        # vision 工具注入
        self._vision = None

        # 物理状态
        self.x_mm: float = 0.0
        self.y_mm: float = 0.0
        self.yaw_deg: float = 0.0
        self._cumulative_odom: float = 0.0     # 累计里程 (mm)

        # 里程计累积
        self.odom_x: float = 0.0
        self.odom_y: float = 0.0
        self.odom_yaw: float = 0.0
        self._backtrack_distance: float = 0.0  # 倒车累计距离

        # 状态机（投影字段，只写不反读）
        self.state = AgentState.IDLE
        self._state_enter_time: float = 0.0
        self._started: bool = False

        # 运行时地图事实（收敛进 RuntimeMap，单一写入者）
        self.runtime_map = RuntimeMap()

        # 目标与路径
        self.current_node: str = "START"

        # APPROACHING / TURNING
        self.approach_deadline: float = 0.0
        self.approach_duration: float = (
            _cfg.get("state_machine.approach_duration_s", 0.3)
        )
        self.turn_start_time: float = 0.0
        self.turn_timeout: float = (
            _cfg.get("state_machine.turn_timeout_s", 2.0)
        )

        # 事件日志
        self.event_log: List[Dict] = []

    # ================================================================
    # 依赖注入
    # ================================================================

    def set_clock(self, clock):
        """注入时钟（统一计时）。实机默认 time.time，仿真注入返回 sim_time 的 callable。"""
        self._clock = clock

    def _now(self) -> float:
        """当前时间（统一走注入的时钟）。"""
        return self._clock()

    # ---- 运行时状态只读视图（实际数据在 runtime_map，写走 runtime_map 方法）----
    @property
    def visited_nodes(self) -> frozenset:
        return self.runtime_map.visited_nodes

    @property
    def blocked_edges(self) -> frozenset:
        return self.runtime_map.blocked_edges

    @property
    def discovered_culverts(self) -> frozenset:
        return self.runtime_map.discovered_culverts

    @property
    def recon_culverts(self) -> frozenset:
        return self.runtime_map.recon_culverts

    @property
    def _culvert_targets(self) -> frozenset:
        return self.runtime_map.culvert_targets

    def set_vision_tools(self, tools):
        """注入 vision 工具实例"""
        self._vision = tools

    def set_culvert_targets(self, edge_ids: set):
        """
        注入需要侦查的涵洞目标边集合（仿真用）。

        真实车不知道赛道有几个涵洞，此集合为空时不强制涵洞完成。
        仿真注入 8 个涵洞边后，结束条件要求全部侦查完。
        """
        self.runtime_map.set_culvert_targets(edge_ids)

    def all_culverts_reconed(self) -> bool:
        """是否所有目标涵洞都已侦查完成（读 recon 集合，非 discovered）"""
        return self.runtime_map.all_culverts_reconed()

    # ================================================================
    # 生命周期
    # ================================================================

    def start(self):
        """启动：投影 GLOBAL_PLANNING → 立即规划 + dequeue 首条 drive。"""
        if self._started:
            return
        self._snap_to_node("START")
        self._log_event("startup", "Agent 启动")
        self._transition_to(AgentState.GLOBAL_PLANNING)
        self._started = True
        # immediate: execute planning + dequeue first edge
        self._do_global_planning()

    def tick(self) -> Optional[TurnCommand]:
        """
        主循环：按队首 kind 分派推进（不再按 self.state 分派）。

        - head is None / 队列空 → 到达节点/待规划 → _on_node_arrival / 规划
        - drive → 里程推进（executor.update 进度驱动）
        - turn → 刷新 current_yaw（R8）+ produce TurnCommand + 超时兜底
        - reverse → DeadEndRecovery 倒车机制
        - culvert_probe → 2 秒侦查延迟
        """
        now = self._now()
        q = self.queue_controller

        # 终态
        if self.state in (AgentState.FINISHED, AgentState.FAILED):
            return TurnCommand(action=TurnAction.STOP,
                               target_node=self.current_node,
                               expected_yaw=self.yaw_deg)

        head = q.peek()
        if head is None or q.is_exhausted():
            if self.state == AgentState.IDLE or not self._started:
                return None
            # 队列空 = 到达节点后待推进 / 待重规划
            self._on_node_arrival()
            return None

        kind = head.kind

        if kind == "drive":
            return self._tick_drive(now, head)

        if kind == "turn":
            return self._tick_turn(now, head)

        if kind == "reverse":
            # BACKTRACK：沿用 DeadEndRecovery 机制（读 executor.current_task）
            self._recovery.tick_backtrack(now)
            return None

        if kind == "culvert_probe":
            self._handle_culvert_recon(now)
            return None

        # checkpoint 靠 on_data_acked / on_rfid_scanned 推进，不靠 tick
        return None

    # ================================================================
    # tick 分派（drive / turn）
    # ================================================================

    def _tick_drive(self, now: float, head: Task) -> Optional[TurnCommand]:
        """drive 里程推进：跟进旧 _tick_edge_executing 的 window_ratio / timeout / stalled。"""
        progress, interrupts = self.executor.update(self._cumulative_odom, now)
        before = self.queue_controller.cursor
        self.queue_controller.step(progress, now)
        if self.queue_controller.cursor > before:
            # drive 推进 → R2/R3 副作用 + 到达节点投影
            self._on_drive_advance(progress)
        return None

    def _on_drive_advance(self, progress):
        """drive 推进回调（等价旧 _tick_edge_executing 的三个 finish(DONE) 分支）。"""
        window_ratio = _cfg.get("state_machine.crossroad_detection_window_ratio", 0.95)
        if progress.timeout:
            # R2 日志 #2
            self._log_event("edge_timeout", "超时, 强行到达")
        elif progress.progress_ratio >= window_ratio:
            # R2 日志 #1
            self._log_event("odom_force_arrive", f"ratio={progress.progress_ratio:.2f}")
        # else: 完成且停滞 —— 旧代码无日志
        # R3: finish(DONE)
        self.executor.finish(EdgeTaskStatus.DONE)
        self._transition_to(AgentState.NODE_ARRIVAL)

    def _tick_turn(self, now: float, head: Task) -> Optional[TurnCommand]:
        """turn 队首：刷新 current_yaw（R8），produce 指令，超时兜底。"""
        # R8：turn 存续期内每 tick 刷新 live yaw，保证 180° 兜底用实时语义
        head.params["current_yaw"] = self.yaw_deg

        # 超时兜底（R1: turn_start_time + turn_timeout → TURNING 会卡死则推进）
        if now - self.turn_start_time > self.turn_timeout:
            self._log_event("turn_timeout", "转弯超时")
            self.queue_controller.advance()
            self._transition_to(AgentState.NODE_ARRIVAL)
            return None

        # stop 分支（R5）：decision 无下一跳 → 直接返回 STOP（不 produce 计算）
        if head.params.get("stop"):
            return TurnCommand(action=TurnAction.STOP,
                               target_node=self.current_node,
                               expected_yaw=self.yaw_deg)

        cmd = self.queue_controller.produce_turn_command()
        if cmd is not None and cmd.action == TurnAction.STOP:
            # D2：180° 掉头 STOP 观测锚点（队列 _determine_turn 无日志，门面补记）
            diff = cmd.expected_yaw - self.yaw_deg
            while diff > 180: diff -= 360
            while diff < -180: diff += 360
            self._log_event("turn_uturn_blocked",
                            f"非法 180° 掉头 diff={diff:.1f} → STOP，禁止伪造 90° 转向")
        return cmd

    # ================================================================
    # 入队原语（队首任务 + 副作用 + 投影）
    # ================================================================

    def _enqueue_drive(self, task):
        """入队一条 drive 边任务（等价旧 executor.start + _transition_to(EDGE_EXECUTING)）。"""
        # R4：喂 executor.start 基准（_start_odom = 当前累计里程）
        self.executor.start(task, self._cumulative_odom)
        self.queue_controller.invalidate_and_replace(
            [Task(kind="drive", trigger="distance_reached",
                  params={"edge_id": task.edge_id})]
        )
        self._transition_to(AgentState.EDGE_EXECUTING)
        self._log_event("edge_start",
                        f"{task.from_node}→{task.to_node} {task.distance_mm:.0f}mm")

    def _resume_drive(self):
        """涵洞侦查结束 → 恢复同一条边的 drive（不重设 executor.start 基准，保 R4）。"""
        self.queue_controller.invalidate_and_replace(
            [Task(kind="drive", trigger="distance_reached", params={})]
        )
        self._transition_to(AgentState.EDGE_EXECUTING)

    def _enqueue_reverse(self, obstacle: bool = False):
        """
        入队 reverse（倒车中断）。等价旧 _transition_to(BACKTRACK)。

        R7：不调 executor.start（保留 executor.current_task 的 from_node/distance_mm
        语义，供 DeadEndRecovery.tick_backtrack 读回退阈值）。
        """
        self.queue_controller.invalidate_and_replace(
            [Task(kind="reverse", trigger="distance_reached", params={})]
        )
        self._backtrack_distance = 0.0
        self._transition_to(AgentState.BACKTRACK)

    def _enqueue_culvert_probe(self):
        """入队 culvert_probe（等价旧 on_culvert_entrance_detected → CULVERT_RECON）。"""
        self.queue_controller.invalidate_and_replace(
            [Task(kind="culvert_probe", trigger="data_acked", params={})]
        )
        # R1：_state_enter_time 由 _transition_to(CULVERT_RECON) 写入，供 2s 延迟
        self._transition_to(AgentState.CULVERT_RECON)

    def _enqueue_turn(self, next_node: str, expected_yaw: float,
                      target_node: str, score: float, stop: bool = False):
        """
        入队 turn 任务（等价旧 _on_approach_done 的 TURNING + turn_start_time）。

        R5：expected_yaw 由门面 _calc_expected_yaw 算好注入；180° 兜底经门面
        _determine_turn 计算（保留 turn_uturn_blocked 日志 + turn 业务日志）。
        """
        current_yaw = self.yaw_deg
        if not stop:
            # 门面 _determine_turn：180° 掉头 → STOP 防御兜底（D2 日志锚点）
            action = self._determine_turn(current_yaw, expected_yaw)
            self._log_event(
                "turn", f"{action.value} → {next_node} "
                        f"(target={target_node}, score={score:.1f})"
            )
        self.queue_controller.invalidate_and_replace(
            [Task(kind="turn", trigger="angle_reached",
                  params={"target_node": next_node,
                          "expected_yaw": expected_yaw,
                          "current_yaw": current_yaw,
                          "stop": stop})]
        )
        self._transition_to(AgentState.TURNING)
        self.turn_start_time = self._now()

    # ================================================================
    # 感知事件接口 (被动接收) —— 入队原语 + 副作用
    # ================================================================

    def on_odom_update(self, odom: OdomUpdate):
        yaw_rad = math.radians(self.yaw_deg)
        world_dx = odom.dx_mm * math.cos(yaw_rad) - odom.dy_mm * math.sin(yaw_rad)
        world_dy = odom.dx_mm * math.sin(yaw_rad) + odom.dy_mm * math.cos(yaw_rad)

        self.x_mm += world_dx
        self.y_mm += world_dy
        self.yaw_deg = (self.yaw_deg + odom.dyaw_deg) % 360.0
        if self.yaw_deg > 180.0:
            self.yaw_deg -= 360.0

        self._cumulative_odom += abs(odom.dy_mm) + abs(odom.dx_mm)
        self.odom_x += world_dx
        self.odom_y += world_dy
        self.odom_yaw += odom.dyaw_deg

        # 倒车距离：BACKTRACK 状态下用带符号的纵向增量衡量倒退量
        if self.state == AgentState.BACKTRACK:
            self._backtrack_distance += abs(odom.dy_mm) + abs(odom.dx_mm)

    def on_road_condition(self, rc: RoadCondition):
        """perception 每帧上报的道路状况"""
        pass  # navigation 不每帧响应，仅记录

    def on_crossroad_detected(self, event: CrossroadEvent):
        """
        perception 中断上报：IPM 检测到路口。
        等价旧 EDGE_EXECUTING → APPROACHING(延时) → _on_approach_done。
        换核后：直接触发 turn 入队原语（decide_at_junction 现场选目标）。
        """
        head = self.queue_controller.peek()
        if head is None or head.kind != "drive":
            return  # 非 drive 阶段不处理路口
        # 距离兜底
        dist = event.distance_mm
        if dist <= 0 or dist > _cfg.get("state_machine.crossroad_max_valid_mm", 2000):
            dist = _cfg.get("state_machine.crossroad_fallback_mm", 300)
        if dist > _cfg.get("state_machine.crossroad_max_distance_mm", 800):
            self._log_event("crossroad_far", f"dist={dist:.0f}")
            return

        # D4 下沉：crossroad 成功日志（旧 :153）
        self._log_event("crossroad", f"dist={dist:.0f} → 路口决策")
        # 等价旧 _on_approach_done：现场选目标 + 入队 turn
        self._decide_and_enqueue_turn()

    def _decide_and_enqueue_turn(self):
        """
        turn 入队原语触发点（= 旧 _on_approach_done，D5）。

        下沉 R5：decision.stop / 死规则 ban 出发区桥都交由 decide_at_junction。
        """
        decision = self.decide_at_junction()
        next_node = decision.next_node

        if not next_node or decision.action == "stop":
            # R5：stop 分支不得入队空 target（_calc_expected_yaw(current_node,"") KeyError）
            self._enqueue_turn(next_node="", expected_yaw=self.yaw_deg,
                               target_node=decision.target_node, score=decision.score,
                               stop=True)
            return

        expected_yaw = self._calc_expected_yaw(self.current_node, next_node)
        self._enqueue_turn(next_node=next_node, expected_yaw=expected_yaw,
                           target_node=decision.target_node, score=decision.score)

    def on_turn_done(self):
        """下位机回传 TURN_DONE → 推进 turn 游标（等价旧退出 TURNING）。"""
        head = self.queue_controller.peek()
        if head is not None and head.kind == "turn":
            self._log_event("turn_done", "下位机确认转弯完成")
            self.queue_controller.advance()
            self._transition_to(AgentState.NODE_ARRIVAL)

    def on_culvert_detected(self, event: CulvertEvent):
        """感知线程推送：检测到涵洞墙壁 → 仅「发现」，不「侦查」。"""
        head = self.queue_controller.peek()
        if head is not None and head.kind == "drive":
            task = self.executor.current_task
            if task:
                self._mark_culvert_discovered(task.edge_id)
            self._log_event("culvert_wall", f"edge={task.edge_id if task else '?'}")

    def _mark_culvert_discovered(self, edge_id: int):
        """
        导航状态单点写入：裁决「某条涵洞边被『发现』（未侦查）」。

        只写「发现」态（edge.has_culvert + discovered_culverts），不写侦查态。
        与 _mark_culvert_reconed 对称，是「发现」语义唯一的写入入口（P0 收口）。
        """
        try:
            edge = self.topo.get_edge_by_id(edge_id)
        except KeyError:
            return
        edge.has_culvert = True
        self.runtime_map.mark_culvert_discovered(edge_id)

    def on_culvert_entrance_detected(self, event: CulvertEvent):
        """感知线程推送：检测到涵洞口（标签1）→ 入队 culvert_probe（CULVERT_RECON）。"""
        head = self.queue_controller.peek()
        if head is not None and head.kind == "drive":
            self._enqueue_culvert_probe()

    def on_obstacle_detected(self, event: ObstacleEvent):
        """感知线程推送：检测到障碍物在车道内 → 封锁边 + 入队 reverse（BACKTRACK）。"""
        head = self.queue_controller.peek()
        if head is None or head.kind != "drive":
            return
        task = self.executor.current_task
        if task:
            self._log_event("obstacle_in_lane", f"dist={event.distance_mm:.0f}mm")
            try:
                edge = self.topo.get_edge(task.from_node, task.to_node)
                edge.is_blocked = True
                self.runtime_map.block_edge(edge.edge_id)
            except KeyError:
                pass
        self.executor.finish(EdgeTaskStatus.FAILED)
        self._log_event("obstacle", "封锁边 → BACKTRACK")
        self._enqueue_reverse()

    def on_rfid_scanned(self, event: RfidEvent):
        """
        RFID 打卡。R6 四伴生副作用 + D6 打卡完直接推进。

        副作用：rfid_error 日志 / has_rfid 静默 return / _snap_to_node 6 字段 /
        「打卡记完即推进」（等价旧 _transition_to(NODE_ARRIVAL) → 直接推进）。
        """
        node_name = event.uid.upper()
        if node_name not in self.topo.nodes:
            # R6 副作用 #1：未知节点失败日志（与 has_rfid 是两条不同分支，不合并）
            self._log_event("rfid_error", f"未知: {node_name}")
            return

        node = self.topo.get_node(node_name)
        if not node.has_rfid:
            # R6 副作用 #2：has_rfid 分支静默 return
            return

        # R6 副作用 #3：_snap_to_node 6 字段写入
        self._snap_to_node(node_name)
        node.is_visited = True
        self.runtime_map.mark_rfid_visited(node_name)
        self._log_event("rfid", f"打卡: {node_name}")

        # D6/R6 副作用 #4：打卡记完直接推进（旧 _transition_to(NODE_ARRIVAL) 的等价）
        # 清空队列 + 投影 NODE_ARRIVAL，下一 tick 由 _on_node_arrival 决定 replan/继续。
        self.queue_controller.invalidate_and_replace([])
        self._transition_to(AgentState.NODE_ARRIVAL)

    # ================================================================
    # 各状态内部逻辑（旧 _cruise 迁移族 → 队首分派 + 副作用）
    # ================================================================

    def _on_node_arrival(self):
        """到达节点后：更新当前节点 → 判断是否需要重规划或回到起点。"""
        # 端口模型：current_node = 刚完成边的终点（executor 仍保留刚完成的任务）
        task = self.executor.current_task
        if task:
            self.current_node = task.to_node
            self._log_event("node_arrival", f"到达 {self.current_node}")

        # 回到 START 且全部任务完成 + 涵洞侦查完成 → 巡逻结束
        if (self.current_node == "START"
                and self.topo.all_missions_completed()
                and self.all_culverts_reconed()):
            self._log_event("return_complete", "回到起点，巡逻完成")
            self._transition_to(AgentState.FINISHED)
            return

        # RFID 完成但涵洞未侦查完，且当前规划已耗尽 → 重规划扫荡剩余涵洞
        if (self.topo.all_missions_completed()
                and not self.all_culverts_reconed()
                and not self.planner.has_next()):
            self._transition_to(AgentState.GLOBAL_PLANNING)
            return

        # 地图未变 → 直接用缓存的下一任务
        if self.planner.should_replan(blocked_edges=self.blocked_edges):
            self._transition_to(AgentState.GLOBAL_PLANNING)
        else:
            self._dequeue_next_edge()

    def _mark_culvert_reconed(self, edge_id: int):
        """
        导航状态单点写入：裁决「某条涵洞边的侦查完成」。

        统一维护地图边状态（edge.has_culvert / edge.is_reconned）与
        导航决策状态（discovered_culverts / recon_culverts）。
        """
        try:
            edge = self.topo.get_edge_by_id(edge_id)
        except KeyError:
            return
        edge.has_culvert = True
        edge.is_reconned = True
        self.runtime_map.mark_culvert_reconed(edge_id)

    def _handle_culvert_recon(self, now: float = None):
        """涵洞侦查：延迟标记 → 恢复执行（等价旧 _handle_culvert_recon）。"""
        if now is None:
            now = self._now()
        recon_duration = _cfg.get("state_machine.culvert_recon_duration_s", 2.0)
        # R1：_state_enter_time 写入依赖（_enqueue_culvert_probe 已投影 CULVERT_RECON）
        if now - self._state_enter_time < recon_duration:
            return  # 模拟侦查耗时

        task = self.executor.current_task
        if task:
            self._mark_culvert_reconed(task.edge_id)

        self._log_event("culvert_done", "侦查完成")
        self._resume_drive()

    def _on_obstacle_stop(self):
        """障碍停车（保留兼容；实际逻辑已在 on_obstacle_detected 内联）。"""
        task = self.executor.current_task
        if task:
            try:
                edge = self.topo.get_edge(task.from_node, task.to_node)
                edge.is_blocked = True
                self.runtime_map.block_edge(edge.edge_id)
            except KeyError:
                pass
        self.executor.finish(EdgeTaskStatus.FAILED)
        self._log_event("obstacle", f"封锁边 → BACKTRACK")
        self._enqueue_reverse()

    def _on_reverse_done(self):
        """倒车到位：清空 reverse 队列 → 投影 GLOBAL_PLANNING（下一步重规划）。"""
        self.queue_controller.invalidate_and_replace([])
        self._transition_to(AgentState.GLOBAL_PLANNING)

    # ================================================================
    # 宏观调度委托
    # ================================================================

    def _do_global_planning(self):
        """全局规划（委托给 StateOrchestrator）"""
        self._orchestrator.do_global_planning()

    def _dequeue_next_edge(self):
        """取下一段边任务（委托给 StateOrchestrator）"""
        self._orchestrator.dequeue_next_edge()

    def decide_at_junction(self):
        """
        路口决策（D-03）：在路口现场选目标 + 规划下一跳。

        死规则（由控制层调度，非路径规划职责）：
          - 巡逻期 ban 出发区桥（START→J_START.P_N）+ 颈通道（N6.P_E→N7.P_W）。
          - 任务全部完成（RFID + 涵洞）时，解除 ban，允许回 START 触发 FINISHED。

        返回 JunctionDecision（action + target_node + next_node + score）。
        """
        junction = self.current_node

        blocked = set(self.blocked_edges)   # 障碍封锁边（持久集）

        # 死规则注入：巡逻期 ban 出发区/颈通道（START→J_START.P_N）；收尾期放行
        if not self._mission_complete_for_return():
            try:
                blocked.add(self.topo.get_edge("START", "J_START.P_N").edge_id)
            except KeyError:
                pass

        return self._junction_decider.decide(
            junction=junction,
            incoming=self.yaw_deg,
            task_queue=self._task_queue,
            blocked_edges=blocked,
        )

    def _mission_complete_for_return(self) -> bool:
        """任务全部完成（应放行回 START 触发 FINISHED）。"""
        return (self.topo.all_missions_completed()
                and self.all_culverts_reconed())

    def _next_is_reverse(self) -> bool:
        """判断下一段是否反向段（委托给 StateOrchestrator）"""
        return self._orchestrator.next_is_reverse()

    def _tick_backtrack(self, now: float):
        """反向巡航回上一个安全节点（委托给 DeadEndRecovery）"""
        self._recovery.tick_backtrack(now)

    # ================================================================
    # 内部工具方法
    # ================================================================

    def _snap_to_node(self, node_name: str):
        coord = NODE_COORDS[node_name]
        self.x_mm = coord["x"]
        self.y_mm = coord["y"]
        self.current_node = node_name
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

    def _calc_expected_yaw(self, from_node: str, to_node: str) -> float:
        a = self.topo.get_node(from_node)
        b = self.topo.get_node(to_node)
        angle = math.degrees(math.atan2(b.x_mm - a.x_mm, b.y_mm - a.y_mm))
        while angle > 180: angle -= 360
        while angle < -180: angle += 360
        return angle

    def _determine_turn(self, current_yaw: float, expected_yaw: float) -> TurnAction:
        diff = expected_yaw - current_yaw
        while diff > 180: diff -= 360
        while diff < -180: diff += 360
        if abs(diff) < _cfg.get("state_machine.turn_straight_deg", 15.0):
            return TurnAction.STRAIGHT
        # Task 12/14：正常规划禁止 180° 原地掉头。规划层已排除掉头路径，此处是防御兜底。
        # 关键：不能把 180°「降级为 90°」——那会让 expected_yaw 仍是 180°，车头朝向与
        # 下一条边不一致（日志说左转、画面实际掉头）。应返回 STOP，触发上层重规划/恢复，
        # 绝不伪造一个方向不符的转向命令。
        if abs(diff) > _cfg.get("state_machine.turn_uturn_deg", 160.0):
            self._log_event("turn_uturn_blocked",
                            f"非法 180° 掉头 diff={diff:.1f} → STOP，禁止伪造 90° 转向")
            return TurnAction.STOP
        return TurnAction.TURN_LEFT if diff > 0 else TurnAction.TURN_RIGHT

    def _transition_to(self, new_state: AgentState, **kwargs):
        """
        投影写出（R1 三副作用）：self.state + _state_enter_time + 条件 turn_start_time
        + transition 日志。只写投影，不做行为分支。
        """
        old = self.state
        self.state = new_state
        self._state_enter_time = self._now()
        if new_state == AgentState.TURNING:
            self.turn_start_time = self._now()
        self._log_event("transition", f"{old.name} → {new_state.name}")

    def _log_event(self, event_type: str, message: str):
        self.event_log.append({
            "timestamp": self._now(),
            "type": event_type,
            "state": self.state.name,
            "message": message,
            "pos": (round(self.x_mm, 1), round(self.y_mm, 1)),
            "yaw": round(self.yaw_deg, 2),
        })

    # ================================================================
    # 查询接口
    # ================================================================

    @property
    def target_node(self) -> str:
        """当前边任务的目标节点"""
        if self.executor.current_task:
            return self.executor.current_task.to_node
        return ""

    @property
    def planned_path(self) -> List[str]:
        """缓存的全局路径节点序列"""
        return self.planner.get_cached_sequence()

    def get_state_name(self) -> str:
        return self.state.name

    def current_head_kind(self) -> Optional[str]:
        """队首任务 kind（供外部按队列驱动推进，不依赖 self.state 反读）。"""
        head = self.queue_controller.peek()
        if head is None or self.queue_controller.is_exhausted():
            return None
        return head.kind

    def current_state(self) -> AgentState:
        """
        当前状态的只读投影（Task 28 第 3 步：队首 Task.kind → AgentState 单向投影）。

        映射：
          drive→EDGE_EXECUTING / turn→TURNING / reverse→BACKTRACK /
          culvert_probe→CULVERT_RECON / checkpoint→NODE_ARRIVAL /
          空队列→GLOBAL_PLANNING（未启动→IDLE）/ 终态→FINISHED/FAILED。

        语义：只「告诉外部现在是什么状态」，单向、只读，**永远不得**成为任何
        内部逻辑的分支依据（禁止内部 `if current_state() == X` 的反读）。
        """
        if self.state in (AgentState.FINISHED, AgentState.FAILED):
            return self.state
        if not self._started:
            return AgentState.IDLE
        head = self.queue_controller.peek()
        if head is None or self.queue_controller.is_exhausted():
            return AgentState.GLOBAL_PLANNING
        kind = head.kind
        if kind == "drive":
            return AgentState.EDGE_EXECUTING
        if kind == "turn":
            return AgentState.TURNING
        if kind == "reverse":
            return AgentState.BACKTRACK
        if kind == "culvert_probe":
            return AgentState.CULVERT_RECON
        if kind == "checkpoint":
            return AgentState.NODE_ARRIVAL
        return AgentState.GLOBAL_PLANNING

    def get_position(self) -> Tuple[float, float, float]:
        return self.x_mm, self.y_mm, self.yaw_deg

    def get_state(self) -> NavigationState:
        return NavigationState(
            agent_state=self.state,
            pose=Pose(x_mm=self.x_mm, y_mm=self.y_mm, yaw_deg=self.yaw_deg),
            current_node=self.current_node,
            target_node=(self.executor.current_task.to_node
                         if self.executor.current_task else ""),
            visited_nodes=list(self.visited_nodes),
            edge_sequence=self.planner.get_cached_sequence(),
            discovered_culverts=list(self.discovered_culverts),
            recon_culverts=list(self.recon_culverts),
        )

    def get_event_log(self) -> List[Dict]:
        return self.event_log[:]
