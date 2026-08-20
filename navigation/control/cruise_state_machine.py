"""
control/cruise_state_machine.py — 边级状态机（CruiseStateMachine）

从 AgentStateMachine 抽出的「纯状态转移」职责：tick 主循环、各状态处理、
感知事件入口（on_xxx）。

门面硬拆（做法 A）：本类通过持有 `agent` 引用访问共享状态（state/x_mm/
blocked_edges/...）与共享工具（_transition_to/_log_event/_snap_to_node/
_calc_expected_yaw/_determine_turn），不拥有状态本身。状态的唯一承载仍是
AgentStateMachine 门面。

后续任务：观察点三段巡航（D-01）、事件队列并发（D-08）、异步确认（D-09）
落地后，本类才演化为设计文档里的完整边级巡航状态机；当前只抽取职责位置，
逻辑不变。
"""
import math
import time
from typing import Any, Optional

from ..contracts import (
    AgentState, EdgeTaskStatus, TurnAction, TurnCommand,
    OdomUpdate, RoadCondition, CrossroadEvent, CulvertEvent,
    ObstacleEvent, RfidEvent,
)
from ..domain.line_graph import edge_heading, has_safe_exit, iter_out_edges
try:
    from ... import config as _cfg
except ImportError:
    import config as _cfg


class CruiseStateMachine:
    """边级状态机 —— 只做状态转移，不做宏观调度（宏观调度归 StateOrchestrator）"""

    def __init__(self, agent: Any):
        self._agent = agent

    # ================================================================
    # 生命周期
    # ================================================================

    def start(self):
        agent = self._agent
        if agent.state != AgentState.IDLE:
            return
        agent._snap_to_node("START")
        agent._log_event("startup", "Agent 启动")
        agent._transition_to(AgentState.GLOBAL_PLANNING)
        # immediate: execute planning + dequeue first edge
        # 注意：_do_global_planning 内部已调用 _dequeue_next_edge，不要重复
        agent._do_global_planning()

    def tick(self) -> Optional[TurnCommand]:
        agent = self._agent
        now = time.time()

        if agent.state == AgentState.IDLE:
            return None

        if agent.state == AgentState.GLOBAL_PLANNING:
            agent._do_global_planning()
            return None

        if agent.state == AgentState.EDGE_EXECUTING:
            return self._tick_edge_executing(now)

        if agent.state == AgentState.APPROACHING:
            if now >= agent.approach_deadline:
                return self._on_approach_done()
            return None

        if agent.state == AgentState.TURNING:
            if now - agent.turn_start_time > agent.turn_timeout:
                agent._log_event("turn_timeout", "转弯超时")
                agent._transition_to(AgentState.NODE_ARRIVAL)
            return None

        if agent.state == AgentState.NODE_ARRIVAL:
            self._on_node_arrival()
            return None

        if agent.state == AgentState.CULVERT_RECON:
            self._handle_culvert_recon(now)
            return None

        if agent.state == AgentState.OBSTACLE_STOP:
            self._on_obstacle_stop()
            return None

        if agent.state == AgentState.BACKTRACK:
            agent._tick_backtrack(now)
            return None

        if agent.state == AgentState.FAILED:
            return TurnCommand(action=TurnAction.STOP,
                               target_node=agent.current_node,
                               expected_yaw=agent.yaw_deg)

        if agent.state == AgentState.FINISHED:
            return TurnCommand(action=TurnAction.STOP,
                               target_node=agent.current_node,
                               expected_yaw=agent.yaw_deg)

        return None

    # ================================================================
    # 感知事件接口 (被动接收)
    # ================================================================

    def on_odom_update(self, odom: OdomUpdate):
        agent = self._agent
        yaw_rad = math.radians(agent.yaw_deg)
        world_dx = odom.dx_mm * math.cos(yaw_rad) - odom.dy_mm * math.sin(yaw_rad)
        world_dy = odom.dx_mm * math.sin(yaw_rad) + odom.dy_mm * math.cos(yaw_rad)

        agent.x_mm += world_dx
        agent.y_mm += world_dy
        agent.yaw_deg = (agent.yaw_deg + odom.dyaw_deg) % 360.0
        if agent.yaw_deg > 180.0:
            agent.yaw_deg -= 360.0

        agent._cumulative_odom += abs(odom.dy_mm) + abs(odom.dx_mm)
        agent.odom_x += world_dx
        agent.odom_y += world_dy
        agent.odom_yaw += odom.dyaw_deg

        # 倒车距离：BACKTRACK 状态下用带符号的纵向增量衡量倒退量
        # （dy_mm 为负 = 倒车，为正 = 继续前进，abs 化后能衡量「实际倒退了多少」）
        if agent.state == AgentState.BACKTRACK:
            agent._backtrack_distance += abs(odom.dy_mm) + abs(odom.dx_mm)

    def on_road_condition(self, rc: RoadCondition):
        """perception 每帧上报的道路状况"""
        pass  # navigation 不每帧响应，仅记录

    def on_crossroad_detected(self, event: CrossroadEvent):
        """
        perception 中断上报：IPM 检测到路口。
        仅 EDGE_EXECUTING 状态处理。
        """
        agent = self._agent
        if agent.state == AgentState.EDGE_EXECUTING:
            # 距离兜底
            dist = event.distance_mm
            if dist <= 0 or dist > _cfg.get("state_machine.crossroad_max_valid_mm", 2000):
                dist = _cfg.get("state_machine.crossroad_fallback_mm", 300)
            if dist > _cfg.get("state_machine.crossroad_max_distance_mm", 800):
                agent._log_event("crossroad_far", f"dist={dist:.0f}")
                return

            agent._transition_to(AgentState.APPROACHING)
            agent.approach_deadline = time.time() + agent.approach_duration
            agent._log_event("crossroad", f"dist={dist:.0f} → APPROACHING")

    def on_turn_done(self):
        """下位机回传 TURN_DONE → 退出 TURNING 状态"""
        agent = self._agent
        if agent.state == AgentState.TURNING:
            agent._log_event("turn_done", "下位机确认转弯完成")
            agent._transition_to(AgentState.NODE_ARRIVAL)

    def on_culvert_detected(self, event: CulvertEvent):
        """感知线程推送：检测到涵洞墙壁（标签3 + 非隧道边）→ 仅「发现」，不「侦查」"""
        agent = self._agent
        if agent.state == AgentState.EDGE_EXECUTING:
            task = agent.executor.current_task
            if task:
                self._mark_culvert_discovered(task.edge_id)
            agent._log_event("culvert_wall", f"edge={task.edge_id if task else '?'}")

    def _mark_culvert_discovered(self, edge_id: int):
        """
        导航状态单点写入：裁决「某条涵洞边被『发现』（未侦查）」。

        只写「发现」态（edge.has_culvert + discovered_culverts），不写侦查态。
        与 _mark_culvert_reconed 对称，是「发现」语义唯一的写入入口（P0 收口）。
        """
        agent = self._agent
        try:
            edge = agent.topo.get_edge_by_id(edge_id)
        except KeyError:
            return
        edge.has_culvert = True
        agent.runtime_map.mark_culvert_discovered(edge_id)

    def on_culvert_entrance_detected(self, event: CulvertEvent):
        """感知线程推送：检测到涵洞口（标签1）"""
        agent = self._agent
        if agent.state == AgentState.EDGE_EXECUTING:
            agent._transition_to(AgentState.CULVERT_RECON)

    def on_obstacle_detected(self, event: ObstacleEvent):
        """感知线程推送：检测到障碍物在车道内"""
        agent = self._agent
        if agent.state == AgentState.EDGE_EXECUTING:
            task = agent.executor.current_task
            if task:
                agent._log_event("obstacle_in_lane", f"dist={event.distance_mm:.0f}mm")
            agent._transition_to(AgentState.OBSTACLE_STOP, obstacle_event=event)

    def on_rfid_scanned(self, event: RfidEvent):
        agent = self._agent
        node_name = event.uid.upper()
        if node_name not in agent.topo.nodes:
            agent._log_event("rfid_error", f"未知: {node_name}")
            return

        node = agent.topo.get_node(node_name)
        if not node.has_rfid:
            return

        agent._snap_to_node(node_name)
        node.is_visited = True
        agent.runtime_map.mark_rfid_visited(node_name)
        agent._log_event("rfid", f"打卡: {node_name}")

        # 无论是否完成所有任务点，都进入 NODE_ARRIVAL
        # NODE_ARRIVAL 会触发 replan，由 replan 决定是继续访问还是返回 START
        agent._transition_to(AgentState.NODE_ARRIVAL)

    # ================================================================
    # 各状态内部逻辑
    # ================================================================

    def _tick_edge_executing(self, now: float) -> Optional[TurnCommand]:
        agent = self._agent
        progress, interrupts = agent.executor.update(agent._cumulative_odom, now)

        # 里程计兜底：超过窗口比例仍未检测到路口 → 强制到达
        window_ratio = _cfg.get("state_machine.crossroad_detection_window_ratio", 0.95)
        if progress.progress_ratio >= window_ratio:
            agent._log_event("odom_force_arrive", f"ratio={progress.progress_ratio:.2f}")
            agent.executor.finish(EdgeTaskStatus.DONE)
            agent._transition_to(AgentState.NODE_ARRIVAL)
            return None

        # 边完成判定
        if progress.timeout:
            agent._log_event("edge_timeout", "超时, 强行到达")
            agent.executor.finish(EdgeTaskStatus.DONE)
            agent._transition_to(AgentState.NODE_ARRIVAL)
            return None

        if progress.progress_ratio >= 1.0 and progress.is_stalled:
            agent.executor.finish(EdgeTaskStatus.DONE)
            agent._transition_to(AgentState.NODE_ARRIVAL)
            return None

        return None

    def _on_approach_done(self) -> TurnCommand:
        agent = self._agent
        agent._transition_to(AgentState.TURNING)
        agent.turn_start_time = time.time()

        # 从 planner 缓存取下一任务来判转向
        next_task = agent.planner.peek_task()
        if next_task is None:
            return TurnCommand(action=TurnAction.STOP,
                               target_node=agent.current_node,
                               expected_yaw=agent.yaw_deg)

        expected_yaw = agent._calc_expected_yaw(
            agent.current_node, next_task.to_node
        )
        action = agent._determine_turn(agent.yaw_deg, expected_yaw)

        agent._log_event("turn", f"{action.value} → {next_task.to_node}")
        return TurnCommand(action=action, target_node=next_task.to_node,
                           expected_yaw=expected_yaw)

    def _on_node_arrival(self):
        """到达节点后：更新当前节点 → 判断是否需要重规划或回到起点"""
        agent = self._agent
        # 端口模型：current_node = 刚完成边的终点（executor 仍保留刚完成的任务）
        task = agent.executor.current_task
        if task:
            agent.current_node = task.to_node
            agent._log_event("node_arrival", f"到达 {agent.current_node}")

        # 回到 START 且全部任务完成 + 涵洞侦查完成 → 巡逻结束
        if (agent.current_node == "START"
                and agent.topo.all_missions_completed()
                and agent.all_culverts_reconed()):
            agent._log_event("return_complete", "回到起点，巡逻完成")
            agent._transition_to(AgentState.FINISHED)
            return

        # RFID 完成但涵洞未侦查完，且当前规划已耗尽 → 重规划扫荡剩余涵洞
        if (agent.topo.all_missions_completed()
                and not agent.all_culverts_reconed()
                and not agent.planner.has_next()):
            agent._transition_to(AgentState.GLOBAL_PLANNING)
            return

        # 地图未变 → 直接用缓存的下一任务
        if agent.planner.should_replan(blocked_edges=agent.blocked_edges):
            agent._transition_to(AgentState.GLOBAL_PLANNING)
        else:
            agent._dequeue_next_edge()

    def _mark_culvert_reconed(self, edge_id: int):
        """
        导航状态单点写入：裁决「某条涵洞边的侦查完成」。

        统一维护地图边状态（edge.has_culvert / edge.is_reconned）与
        导航决策状态（discovered_culverts / recon_culverts）。

        这是这两组状态唯一的写入入口（P0 收口）。sim_engine / 视觉模拟器
        只发事件，不直接写这些字段。
        """
        agent = self._agent
        try:
            edge = agent.topo.get_edge_by_id(edge_id)
        except KeyError:
            return
        edge.has_culvert = True
        edge.is_reconned = True
        agent.runtime_map.mark_culvert_reconed(edge_id)

    def _handle_culvert_recon(self, now: float = None):
        """涵洞侦查：延迟标记 → 恢复执行"""
        agent = self._agent
        if now is None:
            now = time.time()
        recon_duration = _cfg.get("state_machine.culvert_recon_duration_s", 2.0)
        if now - agent._state_enter_time < recon_duration:
            return  # 模拟侦查耗时

        task = agent.executor.current_task
        if task:
            self._mark_culvert_reconed(task.edge_id)

        agent._log_event("culvert_done", "侦查完成")
        agent._transition_to(AgentState.EDGE_EXECUTING)

    def _on_obstacle_stop(self):
        agent = self._agent
        task = agent.executor.current_task
        if task:
            try:
                edge = agent.topo.get_edge(task.from_node, task.to_node)
                edge.is_blocked = True
                agent.runtime_map.block_edge(edge.edge_id)
            except KeyError:
                pass
        agent.executor.finish(EdgeTaskStatus.FAILED)
        agent._log_event("obstacle", f"封锁边 → BACKTRACK")
        agent._backtrack_distance = 0.0  # 重置倒车累计，退出障碍后重新计量
        agent._transition_to(AgentState.BACKTRACK)
