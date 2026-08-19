"""
control/orchestrator.py — 状态编排器（StateOrchestrator）

从 AgentStateMachine 抽出的「宏观调度」职责：全局规划、边队列调度、反向段判定。

门面硬拆（做法 A）：本类通过持有 `agent` 引用访问共享状态（topo/planner/
blocked_edges/current_node/...），不拥有状态本身。状态的唯一承载仍是
AgentStateMachine 门面。

后续任务：目标选择 score（D-04）、权重策略（D-05）、现场 Dijkstra（D-03）
落地后，本类才是完整的「宏观决策枢纽」；当前只抽取职责位置，逻辑不变。
"""
from typing import Any

from ..contracts import AgentState
try:
    from ... import config as _cfg
except ImportError:
    import config as _cfg


class StateOrchestrator:
    """宏观调度职责对象 —— 只负责「去哪里、何时结束、何时重规划」"""

    def __init__(self, agent: Any):
        self._agent = agent

    def do_global_planning(self):
        agent = self._agent
        unvisited = [n for n in agent.topo.nodes
                     if agent.topo.nodes[n].has_rfid
                     and not agent.topo.nodes[n].is_visited]

        # S-14：涵洞边作为「必经边」纳入规划，与 RFID 一起排最优序，不再"先全打卡再扫荡"。
        # 分两档：
        #   1. 已发现未侦查的涵洞边（discovered - recon）—— 车已知位置，顺路侦查。
        #   2. 扫荡兜底：RFID 全完成但仍有未侦查涵洞时，用场景真值强制扫荡剩余边
        #      （含尚未"发现"的，因为结束条件要求全部侦查）。
        required_edges = []
        discovered_unrecon = agent.discovered_culverts - agent.recon_culverts
        if unvisited:
            required_edges = sorted(discovered_unrecon)
        elif agent._culvert_targets and agent.recon_culverts < agent._culvert_targets:
            required_edges = sorted(agent._culvert_targets - agent.recon_culverts)

        result = agent.planner.replan(agent.current_node, unvisited,
                                      blocked_edges=agent.blocked_edges,
                                      extra_targets=None,
                                      entry_heading=agent.yaw_deg,
                                      required_edges=required_edges)
        if not result.edge_tasks:
            if unvisited or agent.current_node != "START":
                agent._log_event("plan_failed",
                                 f"无法到达剩余 {len(unvisited)} 个任务点, blocked={agent.blocked_edges}")
                agent._transition_to(AgentState.FAILED)
                return
            if not agent.all_culverts_reconed():
                agent._log_event("plan_failed",
                                 f"涵洞未侦查完且无法规划扫荡路径, remaining={agent._culvert_targets - agent.recon_culverts}")
                agent._transition_to(AgentState.FAILED)
                return
            agent._transition_to(AgentState.FINISHED)
            return

        if required_edges:
            mode = "sweep" if not unvisited else "concurrent"
            agent._log_event("required_edges",
                             f"涵洞必经边 {len(required_edges)} 条 ({mode}), ids={required_edges[:8]}")

        if result.returning_to_start:
            agent._log_event("returning", "所有任务完成 → 返回 START")

        self.dequeue_next_edge()

    def dequeue_next_edge(self):
        agent = self._agent
        if self.next_is_reverse():
            agent._log_event("reverse_segment", "下一段为反向段 → 进入倒车中断 BACKTRACK")
            agent._backtrack_distance = 0.0
            agent._transition_to(AgentState.BACKTRACK)
            return

        task = agent.planner.next_task()
        if task is None:
            if (agent.current_node == "START"
                    and agent.topo.all_missions_completed()
                    and agent.all_culverts_reconed()):
                agent._log_event("finish", "回到起点且全部任务完成 → FINISHED")
                agent._transition_to(AgentState.FINISHED)
            elif agent.topo.all_missions_completed() and not agent.all_culverts_reconed():
                agent._log_event("replan_sweep", "RFID 完成但涵洞未侦查完 → 扫荡重规划")
                agent._transition_to(AgentState.GLOBAL_PLANNING)
            else:
                agent._log_event("replan_exhausted",
                                 f"路径耗尽但任务未完成 (node={agent.current_node}) → 重规划")
                agent._transition_to(AgentState.GLOBAL_PLANNING)
            return

        agent.executor.start(task, agent._cumulative_odom)
        agent._transition_to(AgentState.EDGE_EXECUTING)
        agent._log_event("edge_start",
                         f"{task.from_node}→{task.to_node} {task.distance_mm:.0f}mm")

    def next_is_reverse(self) -> bool:
        """判断下一段边是否相对车头 180° 反向（需倒车而非掉头）。START 例外。"""
        agent = self._agent
        if agent.current_node == "START":
            return False
        task = agent.planner.peek_task()
        if task is None:
            return False
        try:
            from ..domain.line_graph import turn_angle_degs
            diff = abs(turn_angle_degs(agent.yaw_deg, task.expected_yaw))
            return diff > _cfg.get("state_machine.turn_uturn_deg", 160.0)
        except Exception:
            return False
