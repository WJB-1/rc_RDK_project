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
    AgentState, TurnAction, TurnCommand, Pose, NavigationState,
    OdomUpdate, RoadCondition, CrossroadEvent, CulvertEvent,
    ObstacleEvent, RfidEvent,
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
from .control.cruise_state_machine import CruiseStateMachine
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

        # 时钟（统一计时）：默认真实墙钟；仿真注入 sim_time（见 set_clock）
        self._clock = time.time

        # 门面硬拆：职责对象（做法 A）
        self._recovery = DeadEndRecovery(self)      # 死胡同倒车恢复
        self._orchestrator = StateOrchestrator(self)  # 宏观调度
        self._cruise = CruiseStateMachine(self)      # 边级状态转移

        # 路口决策器（D-03）+ 任务队列适配
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
        self._backtrack_distance: float = 0.0  # 倒车累计距离（进入 BACKTRACK 后）

        # 状态机
        self.state = AgentState.IDLE
        self._state_enter_time: float = 0.0

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
        """启动状态机（委托给 CruiseStateMachine）"""
        self._cruise.start()

    def tick(self) -> Optional[TurnCommand]:
        """主循环（委托给 CruiseStateMachine）"""
        return self._cruise.tick()

    # ================================================================
    # 感知事件接口 (被动接收)
    # ================================================================

    def on_odom_update(self, odom: OdomUpdate):
        """里程计更新（委托给 CruiseStateMachine）"""
        self._cruise.on_odom_update(odom)

    def on_road_condition(self, rc: RoadCondition):
        """道路状况上报（委托给 CruiseStateMachine）"""
        self._cruise.on_road_condition(rc)

    def on_crossroad_detected(self, event: CrossroadEvent):
        """路口检测（委托给 CruiseStateMachine）"""
        self._cruise.on_crossroad_detected(event)

    def on_turn_done(self):
        """转弯完成确认（委托给 CruiseStateMachine）"""
        self._cruise.on_turn_done()

    def on_culvert_detected(self, event: CulvertEvent):
        """涵洞墙壁检测（委托给 CruiseStateMachine）"""
        self._cruise.on_culvert_detected(event)

    def _mark_culvert_discovered(self, edge_id: int):
        """涵洞发现标记（委托给 CruiseStateMachine）"""
        self._cruise._mark_culvert_discovered(edge_id)

    def on_culvert_entrance_detected(self, event: CulvertEvent):
        """涵洞口检测（委托给 CruiseStateMachine）"""
        self._cruise.on_culvert_entrance_detected(event)

    def on_obstacle_detected(self, event: ObstacleEvent):
        """障碍物检测（委托给 CruiseStateMachine）"""
        self._cruise.on_obstacle_detected(event)

    def on_rfid_scanned(self, event: RfidEvent):
        """RFID 打卡（委托给 CruiseStateMachine）"""
        self._cruise.on_rfid_scanned(event)

    # ================================================================
    # 各状态内部逻辑
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

    def _tick_edge_executing(self, now: float) -> Optional[TurnCommand]:
        """边执行 tick（委托给 CruiseStateMachine）"""
        return self._cruise._tick_edge_executing(now)

    def _on_approach_done(self) -> TurnCommand:
        """逼近完成（委托给 CruiseStateMachine）"""
        return self._cruise._on_approach_done()

    def _on_node_arrival(self):
        """到达节点（委托给 CruiseStateMachine）"""
        self._cruise._on_node_arrival()

    def _mark_culvert_reconed(self, edge_id: int):
        """涵洞侦查完成标记（委托给 CruiseStateMachine）"""
        self._cruise._mark_culvert_reconed(edge_id)

    def _handle_culvert_recon(self, now: float = None):
        """涵洞侦查状态处理（委托给 CruiseStateMachine）"""
        self._cruise._handle_culvert_recon(now)

    def _on_obstacle_stop(self):
        """障碍停车（委托给 CruiseStateMachine）"""
        self._cruise._on_obstacle_stop()

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
