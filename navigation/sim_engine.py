"""
navigation/sim_engine.py — 仿真引擎

驱动真实 AgentStateMachine 的仿真主循环:
1. 虚拟里程计注入（沿边方向移动）
2. 视觉发现检查（VisionChecker）
3. RFID 打卡模拟（到达 mission 节点）
4. 转向完成模拟（TURNING 状态处理）
5. 状态推送（WebPushServer）
"""

import math
import time
import threading
from typing import Optional, Callable, Dict, Any, List

from .contracts import (
    AgentState, OdomUpdate, RfidEvent, TurnAction, TurnCommand,
    CulvertEvent, ObstacleEvent, CulvertType, EdgeTask,
)
from .domain.topology import RaceTrackTopology, get_topology
from .domain.config import NODE_COORDS
from .state_machine import AgentStateMachine
from .perception.simulation.scene import SimScene
from .perception.simulation.vision_checker import VisionChecker


class SimEngine:
    """
    仿真引擎 — 驱动 AgentStateMachine 进行完整巡逻仿真。

    使用方式:
        engine = SimEngine(scene, topo, on_state_update)
        engine.run(speed_mms=300.0, speed_multiplier=1.0)
        # 或逐步调试:
        engine.start()
        while not engine.is_done():
            engine.step()
    """

    def __init__(
        self,
        scene: SimScene,
        topo: Optional[RaceTrackTopology] = None,
        on_state_update: Optional[Callable[[dict], None]] = None,
    ):
        """
        Args:
            scene: SimScene (地面真值)
            topo: RaceTrackTopology (None → get_topology())
            on_state_update: Callable[[dict], None] — 状态推送回调
        """
        self._scene = scene
        self._topo = topo or get_topology()
        self._topo.reset_visit_status()

        # 重置边的动态状态（清理之前的仿真残留）
        for edge in self._topo.edges:
            edge.is_blocked = False
            edge.has_culvert = False
            edge.is_reconned = False
            edge.visit_count = 0

        self._agent = AgentStateMachine(self._topo)
        # 注入涵洞目标，使结束条件要求 8 涵洞全部侦查完
        self._agent.set_culvert_targets(set(scene.culvert_edge_ids))
        self._vision = VisionChecker(self._topo, scene, visible_range_mm=400.0)
        self._on_state_update = on_state_update

        # 仿真控制
        self._running = False
        self._paused = False
        self._tick_hz = 20

        # 当前边信息（用于里程计注入）
        self._current_edge_id: int = -1
        self._current_edge_from: str = ""
        self._current_edge_dist: float = 0.0
        self._edge_start_odom: float = 0.0

        # 上一个状态（用于检测状态转移）
        self._prev_state = AgentState.IDLE
        self._last_node: str = "START"

        # [yaw平滑] 显示层航向角：内部 agent.yaw_deg 在路口会瞬时跳变（瞬时转弯），
        # 推送前端时用 _display_yaw 以有限角速度渐进逼近，消除「路口车身瞬间扭头」。
        self._display_yaw: Optional[float] = None
        self._yaw_smooth_max_deg_s: float = 180.0   # 最大角速度 (deg/s)，180 表示 90° 转弯约 0.5s 平滑完成

        # 统计
        self._sim_time: float = 0.0
        self._total_distance: float = 0.0
        self._tick_count: int = 0

        # 后台线程
        self._thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None

    # ================================================================
    # 公开接口 — 运行控制
    # ================================================================

    def run(
        self,
        speed_mms: float = 300.0,
        speed_multiplier: float = 1.0,
        tick_hz: int = 20,
    ):
        """
        启动仿真主循环（同步执行，阻塞当前线程）。

        Args:
            speed_mms: 虚拟车速 (mm/s)，默认 300mm/s
            speed_multiplier: 时间加速倍率 (1.0=实时, 2.0=2倍速)
            tick_hz: 控制频率 (Hz)，默认 20Hz
        """
        self._running = True
        self._paused = False
        self._tick_hz = tick_hz
        dt = 1.0 / tick_hz

        # 启动状态机
        self._agent.start()
        self._prev_state = self._agent.state
        self._push_state()

        while self._running:
            if self._paused:
                time.sleep(0.05)
                continue

            # 检查是否已完成
            if self._agent.state in (AgentState.FINISHED, AgentState.FAILED):
                self._push_state()
                break

            # 执行单步
            self._step_internal(speed_mms, dt)

            # 实时节拍
            self._sim_time += dt
            time.sleep(dt / max(speed_multiplier, 0.01))

        self._running = False

    def run_async(
        self,
        speed_mms: float = 300.0,
        speed_multiplier: float = 1.0,
        tick_hz: int = 20,
    ) -> threading.Thread:
        """
        在后台线程中启动仿真。

        Returns:
            threading.Thread: 仿真线程
        """
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run_thread_target,
            args=(speed_mms, speed_multiplier, tick_hz),
            daemon=True,
            name="SimEngineThread",
        )
        self._thread.start()
        return self._thread

    def _run_thread_target(
        self,
        speed_mms: float,
        speed_multiplier: float,
        tick_hz: int,
    ):
        """后台线程入口"""
        try:
            self.run(speed_mms, speed_multiplier, tick_hz)
        except Exception as e:
            print(f"[SimEngine] 仿真线程异常: {e}")
            self._running = False

    def start(self):
        """
        启动仿真但不进入主循环（用于逐步调试）。
        仅初始化状态机并推送初始状态。
        """
        self._running = True
        self._paused = False
        self._sim_time = 0.0
        self._tick_count = 0

        self._agent.start()
        self._prev_state = self._agent.state
        self._push_state()

    def step(self):
        """
        执行一个仿真 tick（用于逐步调试）。

        Returns:
            bool: True=继续, False=已完成

        Raises:
            RuntimeError: 如果未调用 start() 或状态机未处于活动状态
        """
        if not self._running:
            raise RuntimeError("SimEngine 未启动，请先调用 start()")

        if self._agent.state in (AgentState.FINISHED, AgentState.FAILED):
            self._push_state()
            self._running = False
            return False

        if not self._paused:
            dt = 1.0 / self._tick_hz
            self._step_internal(speed_mms=300.0, dt=dt)
            self._sim_time += dt

        return True

    def stop(self):
        """停止仿真"""
        self._running = False
        if self._stop_event:
            self._stop_event.set()

    def pause(self):
        """暂停仿真"""
        self._paused = True

    def resume(self):
        """恢复仿真"""
        self._paused = False

    def is_done(self) -> bool:
        """仿真是否已完成（FINISHED 或 FAILED）"""
        return self._agent.state in (AgentState.FINISHED, AgentState.FAILED)

    def is_running(self) -> bool:
        """仿真循环是否仍在运行"""
        return self._running

    def is_paused(self) -> bool:
        """仿真是否处于暂停状态"""
        return self._paused

    # ================================================================
    # 公开接口 — 信息查询
    # ================================================================

    def get_summary(self) -> Dict[str, Any]:
        """返回仿真运行摘要统计"""
        mission_progress = self._topo.get_mission_progress()
        nav_state = self._agent.get_state()
        vision_stats = self._vision.get_discovery_stats()

        return {
            "state": self._agent.state.name,
            "sim_time_s": round(self._sim_time, 2),
            "tick_count": self._tick_count,
            "total_distance_mm": round(self._total_distance, 1),
            "current_node": self._agent.current_node,
            "mission_progress": f"{mission_progress[0]}/{mission_progress[1]}",
            "blocked_edges": sorted(self._agent.blocked_edges),
            "discovered_culverts": sorted(self._agent.discovered_culverts),
            "recon_culverts": sorted(self._agent.recon_culverts),
            "vision_stats": vision_stats,
            "event_log": self._agent.get_event_log()[-20:],  # 最近 20 条
        }

    def get_state_snapshot(self) -> Dict[str, Any]:
        """获取当前状态的完整快照（用于 web 面板）"""
        nav_state = self._agent.get_state()
        mission_progress = self._topo.get_mission_progress()

        task: Optional[EdgeTask] = self._agent.executor.current_task
        edge_info = None
        if task is not None:
            edge_info = {
                "edge_id": task.edge_id,
                "from_node": task.from_node,
                "to_node": task.to_node,
                "distance_mm": task.distance_mm,
                "is_tunnel": task.is_tunnel,
            }

        return {
            # 状态机信息
            "agent_state": nav_state.agent_state.name,
            "current_node": self._agent.current_node,
            "target_node": self._agent.target_node,
            "planned_path": self._agent.planned_path,
            "pose": {
                "x_mm": round(self._agent.x_mm, 1),
                "y_mm": round(self._agent.y_mm, 1),
                "yaw_deg": round(self._display_yaw if self._display_yaw is not None else self._agent.yaw_deg, 2),
            },
            # 当前边任务
            "current_edge": edge_info,
            # 进度
            "mission_progress": {
                "visited": mission_progress[0],
                "total": mission_progress[1],
            },
            "visited_nodes": sorted(self._agent.visited_nodes),
            "blocked_edges": sorted(self._agent.blocked_edges),
            "discovered_obstacles": sorted(self._scene.discovered_obstacles),
            "discovered_culverts": sorted(self._agent.discovered_culverts),
            "recon_culverts": sorted(self._agent.recon_culverts),
            # 仿真信息
            "sim_time_s": round(self._sim_time, 2),
            "tick_count": self._tick_count,
            "total_distance_mm": round(self._total_distance, 1),
            "current_edge_id": self._current_edge_id,
            # 事件日志（最近 20 条）
            "event_log": [
                {
                    "t": round(e["timestamp"] - (e.get("_sim_ref") or e["timestamp"]), 2)
                    if "_sim_ref" in e
                    else round(e["timestamp"], 2),
                    "type": e["type"],
                    "state": e["state"],
                    "message": e["message"],
                }
                for e in self._agent.get_event_log()[-20:]
            ],
        }

    # ================================================================
    # 内部方法 — 仿真核心
    # ================================================================

    def _step_internal(self, speed_mms: float, dt: float):
        """
        执行一个仿真 tick 的内部逻辑。

        Args:
            speed_mms: 虚拟车速 (mm/s)
            dt: 时间步长 (s)
        """
        self._tick_count += 1

        # 1. 注入里程计：EDGE_EXECUTING（前进）+ BACKTRACK（倒车动画）
        if self._agent.state == AgentState.EDGE_EXECUTING:
            self._inject_odometry(speed_mms, dt)
        elif self._agent.state == AgentState.BACKTRACK:
            self._inject_backtrack_odometry(speed_mms, dt)

        # 2. 同边视觉检查（仅在 EDGE_EXECUTING 状态）
        if self._agent.state == AgentState.EDGE_EXECUTING:
            self._check_vision_on_edge()

        # 3. Tick 状态机（主要逻辑推进）
        cmd = self._agent.tick()

        # 4. 处理转弯指令（TURNING → 自动完成转弯）
        if cmd is not None and cmd.action != TurnAction.STOP:
            if self._agent.state == AgentState.TURNING:
                self._handle_turn_command(cmd)
                self._agent.on_turn_done()

        # 5. 处理节点到达
        if self._agent.state == AgentState.NODE_ARRIVAL:
            self._handle_node_arrival()

        # 6. 更新边跟踪信息
        self._update_edge_tracking()

        # 7. 推送状态到 UI
        self._push_state()

        # 8. 检测状态变化（日志）
        if self._agent.state != self._prev_state:
            self._prev_state = self._agent.state

    def _inject_odometry(self, speed_mms: float, dt: float):
        """
        注入虚拟里程计数据，并沿当前边做位置插值（替代 yaw 积分，杜绝漂移）。

        关键：不再靠 yaw 旋转累积位置（那样会有累积误差导致穿模/飘出地图），
        而是直接用"当前边的 from_node→to_node"做线性插值定位小车。
        """
        step_mm = speed_mms * dt
        self._total_distance += step_mm

        # 创建里程计更新（累计里程计仍用 dy_mm，供 executor 进度跟踪）
        odom = OdomUpdate(
            dx_mm=0.0,
            dy_mm=step_mm,
            dyaw_deg=0.0,
            timestamp=self._sim_time,
        )
        self._agent.on_odom_update(odom)

        # 沿边插值定位：用当前边任务计算精确位置（不漂移）
        self._interpolate_position_along_edge()

    def _inject_backtrack_odometry(self, speed_mms: float, dt: float):
        """
        BACKTRACK 倒车动画：车沿当前边反向移动（车头方向不变，指向原 to_node）。

        与 _inject_odometry 的区别：
        - 里程为负（dy_mm 为负），让 _backtrack_distance 累积、位置后退。
        - 位置沿边反向插值：从 to_node 往 from_node 退，车头 yaw 保持朝 to_node。
        """
        step_mm = speed_mms * dt
        self._total_distance += step_mm

        odom = OdomUpdate(
            dx_mm=0.0,
            dy_mm=-step_mm,   # 负向 = 倒车
            dyaw_deg=0.0,
            timestamp=self._sim_time,
        )
        self._agent.on_odom_update(odom)
        self._interpolate_position_backwards()

    def _interpolate_position_along_edge(self):
        """
        沿当前边做线性插值，覆盖 agent 位置为边上的精确点。
        消除 yaw 积分累积误差，确保小车始终在赛道边线上，不穿模、不飘出地图。
        """
        task = self._agent.executor.current_task
        if task is None:
            return
        try:
            a = self._topo.get_node(task.from_node)
            b = self._topo.get_node(task.to_node)
        except KeyError:
            return
        # 已走距离（相对该边起点）。
        # 关键：必须与 EdgeExecutor 用同一个「边起点里程」基准 —— executor._start_odom
        # 是在 _dequeue_next_edge 里、里程注入之前设置的，基准正确。
        # 若用独立的 _edge_start_odom（在里程注入之后才更新）会错位一个步长，
        # 导致每条边结束前车被提前拉回起点，产生「向后瞬移一帧」。
        car_pos = self._agent._cumulative_odom - self._agent.executor._start_odom
        edge_len = task.distance_mm
        if edge_len <= 0:
            return
        ratio = max(0.0, min(1.0, car_pos / edge_len))
        self._agent.x_mm = a.x_mm + (b.x_mm - a.x_mm) * ratio
        self._agent.y_mm = a.y_mm + (b.y_mm - a.y_mm) * ratio
        # yaw 对齐边方向
        dx = b.x_mm - a.x_mm
        dy = b.y_mm - a.y_mm
        self._agent.yaw_deg = math.degrees(math.atan2(dx, dy))
        while self._agent.yaw_deg > 180:
            self._agent.yaw_deg -= 360
        while self._agent.yaw_deg < -180:
            self._agent.yaw_deg += 360

    def _interpolate_position_backwards(self):
        """
        倒车动画：车沿当前边反向插值，位置从 to_node 往 from_node 退，
        车头 yaw 保持朝向 to_node（倒车不转向）。

        用 _backtrack_distance（已倒回的距离）计算位置：
        已倒回比例 = _backtrack_distance / edge_len；位置 = to_node - 方向 * 已倒回。
        """
        task = self._agent.executor.current_task
        if task is None:
            return
        try:
            a = self._topo.get_node(task.from_node)   # 安全节点（起点）
            b = self._topo.get_node(task.to_node)     # 障碍/支路末端
        except KeyError:
            return
        edge_len = task.distance_mm
        if edge_len <= 0:
            return
        # 已倒回比例：0=在 to_node（末端），1=回到 from_node（安全节点）
        ratio = max(0.0, min(1.0, self._agent._backtrack_distance / edge_len))
        # 从 to_node 往 from_node 退
        self._agent.x_mm = b.x_mm + (a.x_mm - b.x_mm) * ratio
        self._agent.y_mm = b.y_mm + (a.y_mm - b.y_mm) * ratio
        # 车头保持朝 to_node（原前进方向），倒车不改变朝向
        dx = b.x_mm - a.x_mm
        dy = b.y_mm - a.y_mm
        self._agent.yaw_deg = math.degrees(math.atan2(dx, dy))
        while self._agent.yaw_deg > 180:
            self._agent.yaw_deg -= 360
        while self._agent.yaw_deg < -180:
            self._agent.yaw_deg += 360

    def _check_vision_on_edge(self):
        """
        在边上行驶时的视觉检查。
        检查当前边是否有新发现的障碍物或涵洞。
        """
        task = self._agent.executor.current_task
        if task is None:
            return

        # 计算车辆在边上的位置（从 from_node 起始）
        # 与 _interpolate_position_along_edge 统一用 executor._start_odom 基准
        car_pos_mm = self._agent._cumulative_odom - self._agent.executor._start_odom
        if car_pos_mm < 0:
            car_pos_mm = 0.0

        try:
            events = self._vision.check_on_edge(
                car_position_mm=car_pos_mm,
                edge_id=task.edge_id,
                from_node=task.from_node,
            )
        except Exception:
            return

        for event in events:
            if isinstance(event, ObstacleEvent):
                self._agent.on_obstacle_detected(event)
            elif isinstance(event, CulvertEvent):
                # 涵洞入口检测：判断是否是侧面涵洞
                # 侧面涵洞（路口附近）用 on_culvert_detected，
                # 正前方涵洞入口用 on_culvert_entrance_detected
                if event.culvert_type == CulvertType.FRONT:
                    self._agent.on_culvert_entrance_detected(event)
                else:
                    self._agent.on_culvert_detected(event)

        # 探索检测：小车经过已发现的涵洞位置 → 标记为"已探索"
        self._check_culvert_recon(task, car_pos_mm)

    def _check_culvert_recon(self, task, car_pos_mm: float):
        """
        涵洞探索（侦查）检测。

        语义：涵洞"发现"（vision 检测到）与"探索"（小车经过时实际侦查）是两个阶段。
        当小车沿边行驶，越过某涵洞的 offset 位置时，若该涵洞已发现但未探索，则标记为已探索。

        注意：offset 相对 edge.node_a 计量。若 task 从 node_b 进入，需换算
        car_pos 为"距 node_a 的距离"，否则方向反了导致漏判。
        """
        if task is None:
            return
        eid = task.edge_id
        if eid not in self._scene.discovered_culverts:
            return  # 还没发现
        if eid in self._scene.recon_culverts:
            return  # 已探索过
        offset = self._scene.culvert_offsets.get(eid)
        if offset is None:
            return
        # 换算方向，判断是否「驶过」涵洞 offset。
        # offset 相对 edge.node_a 计量。
        #   - 从 node_a 端进入：pos_from_a 递增，驶过 = pos_from_a >= offset
        #   - 从 node_b 端进入：pos_from_a 递减（edge_len → 0），驶过 = pos_from_a <= offset
        edge = self._topo.get_edge_by_id(eid)
        from_node_b = (task.from_node == edge.node_b)
        passed = False
        if from_node_b:
            pos_from_a = edge.distance_mm - car_pos_mm
            passed = pos_from_a <= offset
        else:
            pos_from_a = car_pos_mm
            passed = pos_from_a >= offset

        if passed:
            # 驶过涵洞 = 侦查完成 = 必然也「发现」了（recon ⊆ discovered）
            # 地面真值（scene.discovered/recon）由仿真器维护：这是「客观世界事实」。
            self._scene.recon_culverts.add(eid)
            self._scene.discovered_culverts.add(eid)
            # 导航状态（agent.* / edge.*）归 state_machine 单一写入（P0 收口）：
            # 只发事件到唯一裁决入口 _mark_culvert_reconed，不再越权直写。
            self._agent._mark_culvert_reconed(eid)
            self._agent._log_event(
                "culvert_recon",
                f"涵洞已侦查 (edge={eid}, pos={car_pos_mm:.0f}mm)"
            )

    def _handle_turn_command(self, cmd: TurnCommand):
        """
        处理转弯指令：更新 agent 的航向角。
        仿真中转弯是瞬时的（实际硬件上由下位机执行）。
        """
        self._agent.yaw_deg = cmd.expected_yaw
        self._agent.odom_yaw = cmd.expected_yaw
        self._agent._log_event(
            "turn_execute",
            f"转向 {cmd.action.value} → yaw={cmd.expected_yaw:.1f}",
        )

    def _handle_node_arrival(self):
        """
        处理节点到达事件：
        0. 吸附小车位置到节点坐标（消除里程计积分累积误差，防止穿模/飘出地图）
        1. 如果是 mission 节点且有 RFID → 模拟 RFID 打卡
        2. 路口侧视：检查相邻边上的涵洞
        """
        node = self._agent.current_node

        # 里程兜底/边完成时，current_node 尚未更新为「刚完成边的终点」，
        # 导致 _snap_to_node 把小车拉回旧节点坐标，产生「向后瞬移一帧」。
        # 真正的到达节点 = executor 刚完成边的 to_node，用它做吸附与后续处理。
        task = self._agent.executor.current_task
        arrival_node = task.to_node if task is not None else node

        # 0. 位置吸附到节点坐标（junction/port/mission 都吸附，消除漂移）
        if arrival_node in self._topo.nodes:
            self._agent._snap_to_node(arrival_node)
            self._agent.current_node = arrival_node

        # 1. RFID 打卡模拟
        if arrival_node in self._topo.nodes:
            topo_node = self._topo.get_node(arrival_node)
            if topo_node.has_rfid:
                rfid_event = RfidEvent(
                    uid=arrival_node,
                    node_name=arrival_node,
                    timestamp=self._sim_time,
                )
                self._agent.on_rfid_scanned(rfid_event)

        # 2. 路口侧视检查：检查相邻边上的涵洞
        try:
            events = self._vision.check_at_node(arrival_node)
        except Exception:
            events = []

        for event in events:
            # 对路口发现的涵洞，查找对应的边 ID 并标记
            if isinstance(event, CulvertEvent):
                self._handle_culvert_at_node(arrival_node, event)

    def _handle_culvert_at_node(self, node_name: str, event: CulvertEvent):
        """
        处理路口节点处发现的涵洞。
        遍历该节点的所有相邻边，标记有涵洞未发现的边。

        语义（2026-08-14 修正）：路口侧视只「发现」，不「侦查」。
        只记 discovered_culverts + 边 has_culvert；recon_culverts / is_reconned
        由真正驶过该边的 _check_culvert_recon 或 CULVERT_RECON 才写。
        """
        for edge in self._topo.get_neighbors(node_name):
            if edge.edge_id in self._scene.culvert_edge_ids:
                # 关键修复：守卫依赖 edge.has_culvert，不能依赖 scene.discovered_culverts，
                # 因为 check_at_node 已先行写入 scene.discovered，若用它守卫会导致
                # edge.has_culvert / agent.discovered 永远不设置，「发现」态被跳过。
                # 导航状态（edge.has_culvert / agent.discovered）归 state_machine
                # 单一写入（P0 收口）：只发事件到 _mark_culvert_discovered。
                was_discovered = edge.edge_id in self._agent.discovered_culverts
                self._agent._mark_culvert_discovered(edge.edge_id)
                if not was_discovered:
                    self._agent._log_event(
                        "culvert_intersection",
                        f"路口 {node_name} 侧视发现边 {edge.node_a}-{edge.node_b} 涵洞",
                    )
            if edge.edge_id in self._scene.culvert_edge_ids:
                # 确保 scene 中也标记为已发现（地面真值，仿真器维护）
                self._scene.discovered_culverts.add(edge.edge_id)

    def _update_edge_tracking(self):
        """
        更新当前边跟踪信息。
        当边任务变化时，刷新 _current_edge_* 系列字段。
        """
        task = self._agent.executor.current_task

        if task is None:
            self._current_edge_id = -1
            self._current_edge_from = ""
            self._current_edge_dist = 0.0
            return

        new_edge_id = task.edge_id
        if new_edge_id != self._current_edge_id:
            # 进入新边，更新跟踪信息
            self._current_edge_id = new_edge_id
            self._current_edge_from = task.from_node
            self._current_edge_dist = task.distance_mm
            self._edge_start_odom = self._agent._cumulative_odom

            # 记录经过次数
            try:
                edge = self._topo.get_edge(task.from_node, task.to_node)
                edge.visit_count += 1
            except KeyError:
                pass

    def _push_state(self):
        """
        推送当前状态到外部回调（如 WebPushServer）。
        """
        if self._on_state_update is None:
            return

        try:
            # 先推进显示层 yaw 平滑，再取快照
            self._advance_display_yaw()
            snapshot = self.get_state_snapshot()
            self._on_state_update(snapshot)
        except Exception as e:
            # 状态推送失败不影响仿真，但打印以便调试
            import traceback
            traceback.print_exc()

    def _advance_display_yaw(self):
        """
        显示层 yaw 平滑：以有限角速度把 _display_yaw 逼近内部 agent.yaw_deg。

        内部 agent.yaw_deg 在路口到达时会因「瞬时转弯」直接跳到目标航向，
        若直接推给前端，车身图标会瞬间旋转 90°/180°，视觉上表现为「路口瞬移」。
        这里用 dt 限幅的角速度渐进逼近，让朝向平滑过渡。
        """
        target = self._agent.yaw_deg
        if self._display_yaw is None:
            self._display_yaw = target
            return
        # 角度差归一化到 (-180, 180]
        diff = target - self._display_yaw
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        # 本 tick 允许的最大角位移
        dt = 1.0 / self._tick_hz
        max_step = self._yaw_smooth_max_deg_s * dt
        if abs(diff) <= max_step:
            self._display_yaw = target
        else:
            self._display_yaw += max_step if diff > 0 else -max_step
        # 归一化到 (-180, 180]
        while self._display_yaw > 180.0:
            self._display_yaw -= 360.0
        while self._display_yaw < -180.0:
            self._display_yaw += 360.0

    # ================================================================
    # 属性访问（只读）
    # ================================================================

    @property
    def agent(self) -> AgentStateMachine:
        """获取内部 AgentStateMachine 实例（只读访问）"""
        return self._agent

    @property
    def topo(self) -> RaceTrackTopology:
        """获取赛道拓扑引用"""
        return self._topo

    @property
    def scene(self) -> SimScene:
        """获取仿真场景引用"""
        return self._scene

    @property
    def vision(self) -> VisionChecker:
        """获取视觉检查器引用"""
        return self._vision

    @property
    def sim_time(self) -> float:
        """仿真时间 (s)"""
        return self._sim_time

    @property
    def total_distance(self) -> float:
        """累计行驶距离 (mm)"""
        return self._total_distance

    @property
    def current_edge_id(self) -> int:
        """当前所在边的 ID (-1 表示无)"""
        return self._current_edge_id
