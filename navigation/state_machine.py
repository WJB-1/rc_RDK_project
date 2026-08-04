"""
navigation/state_machine.py — Agent 核心控制状态机 (V3.0)

状态流转:
  IDLE → GLOBAL_PLANNING → EDGE_EXECUTING ⇄ APPROACHING → TURNING → NODE_ARRIVAL
                             ↑                    ↓                    │
                             │              CULVERT_RECON               │
                             │                    ↓                    │
                             │              EDGE_EXECUTING (恢复)       │
                             │                                         │
                             │              OBSTACLE_STOP               │
                             │                    ↓                    │
                             │              BACKTRACK                   │
                             │                    ↓                    │
                             └──────────────── GLOBAL_PLANNING ←───────┘
"""

import math
import time
from typing import Dict, List, Tuple, Optional

from .contracts import (
    AgentState, EdgeTask, EdgeTaskStatus, TurnAction, TurnCommand,
    Pose, OdomUpdate, RoadCondition, CrossroadEvent, CulvertEvent,
    ObstacleEvent, RfidEvent, NavigationState, CulvertReconResult,
    MapEdgeDynamic, CulvertType,
)
from .map_topology import RaceTrackTopology, get_topology
from .map_oracle import MapOracle
from .map_config import NODE_COORDS
from .path_planner import PathPlanner
from .edge_executor import EdgeExecutor
try:
    from .. import config as _cfg
except ImportError:
    import config as _cfg


class AgentStateMachine:
    """
    Agent 导航大脑状态机 (V3.0)

    职责:
    1. 维护车辆物理坐标与拓扑定位
    2. 管理边级任务执行
    3. 响应感知事件，进行状态切换
    4. 调度 vision 工具进行涵洞/障碍物检测
    """

    def __init__(self, topology: RaceTrackTopology = None):
        self.topo = topology or get_topology()
        self.oracle = MapOracle(self.topo)
        self.planner = PathPlanner(self.oracle, self.topo)
        self.executor = EdgeExecutor()

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

        # 状态机
        self.state = AgentState.IDLE
        self._state_enter_time: float = 0.0

        # 目标与路径
        self.current_node: str = "START"
        self.visited_nodes: set = set()
        self.blocked_edges: set = set()
        self.found_culverts: set = set()

        # APPROACHING / TURNING
        self.approach_deadline: float = 0.0
        self.approach_duration: float = (
            _cfg.get("state_machine.turn_finish_threshold_mm", 50.0) / 1000.0
        )
        self.turn_start_time: float = 0.0
        self.turn_timeout: float = (
            _cfg.get("state_machine.turn_finish_threshold_mm", 50.0) / 1000.0
            + _cfg.get("state_machine.turn_timeout_s", 2.0)
        )

        # 事件日志
        self.event_log: List[Dict] = []

    # ================================================================
    # 依赖注入
    # ================================================================

    def set_vision_tools(self, tools):
        """注入 vision 工具实例"""
        self._vision = tools

    # ================================================================
    # 生命周期
    # ================================================================

    def start(self):
        if self.state != AgentState.IDLE:
            return
        self._snap_to_node("START")
        self._log_event("startup", "Agent 启动")
        self._transition_to(AgentState.GLOBAL_PLANNING)
        # immediate: execute planning + dequeue first edge
        self._do_global_planning()
        self._dequeue_next_edge()

    def tick(self) -> Optional[TurnCommand]:
        now = time.time()

        if self.state == AgentState.IDLE:
            return None

        if self.state == AgentState.GLOBAL_PLANNING:
            self._do_global_planning()
            return None

        if self.state == AgentState.EDGE_EXECUTING:
            return self._tick_edge_executing(now)

        if self.state == AgentState.APPROACHING:
            if now >= self.approach_deadline:
                return self._on_approach_done()
            return None

        if self.state == AgentState.TURNING:
            if now - self.turn_start_time > self.turn_timeout:
                self._log_event("turn_timeout", "转弯超时")
                self._transition_to(AgentState.NODE_ARRIVAL)
            return None

        if self.state == AgentState.NODE_ARRIVAL:
            self._on_node_arrival()
            return None

        if self.state == AgentState.CULVERT_RECON:
            # 侦查子状态：占位，立即退出
            self._handle_culvert_recon()
            return None

        if self.state == AgentState.OBSTACLE_STOP:
            self._on_obstacle_stop()
            return None

        if self.state == AgentState.BACKTRACK:
            self._tick_backtrack(now)
            return None

        if self.state == AgentState.FINISHED:
            return TurnCommand(action=TurnAction.STOP,
                               target_node=self.current_node,
                               expected_yaw=self.yaw_deg)

        return None

    # ================================================================
    # 感知事件接口 (被动接收)
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

    def on_road_condition(self, rc: RoadCondition):
        """perception 每帧上报的道路状况"""
        pass  # navigation 不每帧响应，仅记录

    def on_crossroad_detected(self, event: CrossroadEvent):
        """
        perception 中断上报：IPM 检测到路口。
        仅 EDGE_EXECUTING 状态处理。
        """
        if self.state == AgentState.EDGE_EXECUTING:
            # 距离兜底
            dist = event.distance_mm
            if dist <= 0 or dist > _cfg.get("state_machine.crossroad_max_valid_mm", 2000):
                dist = _cfg.get("state_machine.crossroad_fallback_mm", 300)
            if dist > _cfg.get("state_machine.crossroad_max_distance_mm", 800):
                self._log_event("crossroad_far", f"dist={dist:.0f}")
                return

            self._transition_to(AgentState.APPROACHING)
            self.approach_deadline = time.time() + self.approach_duration
            self._log_event("crossroad", f"dist={dist:.0f} → APPROACHING")

    def on_rfid_scanned(self, event: RfidEvent):
        node_name = event.uid.upper()
        if node_name not in self.topo.nodes:
            self._log_event("rfid_error", f"未知: {node_name}")
            return

        node = self.topo.get_node(node_name)
        if not node.has_rfid:
            return

        self._snap_to_node(node_name)
        node.is_visited = True
        self.visited_nodes.add(node_name)
        self._log_event("rfid", f"打卡: {node_name}")

        if self.topo.all_missions_completed():
            self._transition_to(AgentState.FINISHED)
            return

        self._transition_to(AgentState.NODE_ARRIVAL)

    # ================================================================
    # 各状态内部逻辑
    # ================================================================

    def _do_global_planning(self):
        unvisited = [n for n in self.topo.nodes
                     if self.topo.nodes[n].node_type == "mission"
                     and not self.topo.nodes[n].is_visited]

        if not unvisited:
            self._transition_to(AgentState.FINISHED)
            return

        self.planner.replan(self.current_node, unvisited,
                            blocked_edges=self.blocked_edges)
        self._dequeue_next_edge()

    def _dequeue_next_edge(self):
        task = self.planner.next_task()
        if task is None:
            self._transition_to(AgentState.FINISHED)
            return
        self.executor.start(task, self._cumulative_odom)
        self._transition_to(AgentState.EDGE_EXECUTING)
        self._log_event("edge_start",
                        f"{task.from_node}→{task.to_node} {task.distance_mm:.0f}mm")

    def _tick_edge_executing(self, now: float) -> Optional[TurnCommand]:
        progress, interrupts = self.executor.update(self._cumulative_odom, now)

        # 主动调用 vision 工具检测（仅在有 vision 注入时）
        if self._vision is not None:
            # 涵洞检测（始终运行，传入隧道上下文）
            culvert = self._vision.detect_culvert(
                frame=None,  # 由外部传入，这里通过 interrupts 机制
                is_tunnel=self.executor.current_task.is_tunnel
                if self.executor.current_task else False
            )
            if culvert and culvert.detected:
                self._transition_to(AgentState.CULVERT_RECON,
                                    culvert_event=CulvertEvent(
                                        culvert_type=CulvertType.FRONT,
                                        local_x_mm=culvert.local_x_mm,
                                        local_y_mm=culvert.local_y_mm,
                                        confidence=culvert.confidence,
                                    ))
                return None

            # 障碍物检测
            obstacle_enable = _cfg.get("state_machine.edge_obstacle_enable_ratio", 0.3)
            if progress.progress_ratio > obstacle_enable:
                obstacle = self._vision.detect_obstacle(frame=None)
                if obstacle and obstacle.detected:
                    event = ObstacleEvent(
                        distance_mm=obstacle.distance_mm,
                        confidence=obstacle.confidence,
                    )
                    self._transition_to(AgentState.OBSTACLE_STOP,
                                        obstacle_event=event)
                    return None

        # 边完成判定
        if progress.timeout:
            self._log_event("edge_timeout", f"超时, 强行到达")
            self.executor.finish(EdgeTaskStatus.DONE)
            self._transition_to(AgentState.NODE_ARRIVAL)
            return None

        if progress.progress_ratio >= 1.0 and progress.is_stalled:
            self.executor.finish(EdgeTaskStatus.DONE)
            self._transition_to(AgentState.NODE_ARRIVAL)
            return None

        return None  # 巡航中不下发动作

    def _on_approach_done(self) -> TurnCommand:
        self._transition_to(AgentState.TURNING)
        self.turn_start_time = time.time()

        # 从 planner 缓存取下一任务来判转向
        next_task = self.planner.peek_task()
        if next_task is None:
            return TurnCommand(action=TurnAction.STOP,
                               target_node=self.current_node,
                               expected_yaw=self.yaw_deg)

        expected_yaw = self._calc_expected_yaw(
            self.current_node, next_task.to_node
        )
        action = self._determine_turn(self.yaw_deg, expected_yaw)

        self._log_event("turn", f"{action.value} → {next_task.to_node}")
        return TurnCommand(action=action, target_node=next_task.to_node,
                           expected_yaw=expected_yaw)

    def _on_node_arrival(self):
        """到达节点后：更新当前节点 → 判断是否需要重规划"""
        next_task = self.planner.peek_task()
        if next_task:
            self.current_node = next_task.to_node
            self._log_event("node_arrival", f"到达 {self.current_node}")

        # 地图未变 → 直接用缓存的下一任务
        if self.planner.should_replan(blocked_edges=self.blocked_edges):
            self._transition_to(AgentState.GLOBAL_PLANNING)
        else:
            self._dequeue_next_edge()

    def _handle_culvert_recon(self):
        """涵洞侦查：标记 → 恢复执行"""
        task = self.executor.current_task
        if task:
            try:
                edge = self.topo.get_edge(task.from_node, task.to_node)
                edge.has_culvert = True
                self.found_culverts.add(edge.edge_id)
            except KeyError:
                pass

        self._log_event("culvert_done", "侦查完成")
        self._transition_to(AgentState.EDGE_EXECUTING)

    def _on_obstacle_stop(self):
        task = self.executor.current_task
        if task:
            try:
                edge = self.topo.get_edge(task.from_node, task.to_node)
                edge.is_blocked = True
                self.blocked_edges.add(edge.edge_id)
            except KeyError:
                pass
        self.executor.finish(EdgeTaskStatus.FAILED)
        self._log_event("obstacle", f"封锁边 → BACKTRACK")
        self._transition_to(AgentState.BACKTRACK)

    def _tick_backtrack(self, now: float):
        """反向巡航回上一个安全节点"""
        task = self.executor.current_task
        if task:
            reverse_dist = abs(self._cumulative_odom - self.executor._start_odom)
            if reverse_dist >= task.distance_mm * _cfg.get("state_machine.edge_backtrack_ratio", 0.8):
                self._snap_to_node(task.from_node)
        self._transition_to(AgentState.GLOBAL_PLANNING)

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
        if abs(diff) > _cfg.get("state_machine.turn_uturn_deg", 160.0):
            return TurnAction.UTURN
        return TurnAction.TURN_LEFT if diff > 0 else TurnAction.TURN_RIGHT

    def _transition_to(self, new_state: AgentState, **kwargs):
        old = self.state
        self.state = new_state
        self._state_enter_time = time.time()
        if new_state == AgentState.TURNING:
            self.turn_start_time = time.time()
        self._log_event("transition", f"{old.name} → {new_state.name}")

    def _log_event(self, event_type: str, message: str):
        self.event_log.append({
            "timestamp": time.time(),
            "type": event_type,
            "state": self.state.name,
            "message": message,
            "pos": (round(self.x_mm, 1), round(self.y_mm, 1)),
            "yaw": round(self.yaw_deg, 2),
        })

    # ================================================================
    # 查询接口
    # ================================================================

    def get_state_name(self) -> str:
        return self.state.name

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
        )

    def get_event_log(self) -> List[Dict]:
        return self.event_log[:]
