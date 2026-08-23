"""
planning/deadend_recovery.py — 死胡同倒车恢复（DeadEndRecovery）

从 AgentStateMachine._tick_backtrack 抽出的「倒车脱困」职责。

门面硬拆（做法 A）：本类通过持有 `agent` 引用访问共享状态（state/x_mm/
blocked_edges/...），不拥有状态本身。状态的唯一承载仍是 AgentStateMachine
门面（见 docs/debt/8.18_debt.md §四.5 的 P-18 说明）。

当前仍是 R1/R2 近似（edge_backtrack_ratio 阈值），倒车图 Dijkstra（D-06）
留待后续任务。
"""
from typing import Any

from ..contracts import AgentState
from ..domain.line_graph import edge_heading, has_safe_exit, iter_out_edges
try:
    from ... import config as _cfg
except ImportError:
    import config as _cfg


class DeadEndRecovery:
    """
    死胡同倒车恢复 —— 只负责 BACKTRACK 状态的 tick 逻辑。

    agent：AgentStateMachine 门面引用，用于访问共享字段（executor / yaw /
    blocked_edges / topo）与调用 _snap_to_node / _transition_to / _log_event。
    """

    def __init__(self, agent: Any):
        self._agent = agent

    def tick_backtrack(self, now: float):
        """反向巡航回上一个安全节点（带倒车动画的多 tick 中断处理程序）

        障碍物与涵洞支路退出都属于「死胡同」——车头不能原地掉头，只能倒车退回。
        BACKTRACK 状态持多个 tick：sim_engine 每 tick 注入负向里程（倒车），
        _backtrack_distance 累积；到达 edge_backtrack_ratio 阈值才 snap 回
        from_node 并做 R1/R2 判定，最后返回正常规划。
        """
        agent = self._agent
        task = agent.executor.current_task
        if task:
            threshold = task.distance_mm * _cfg.get("state_machine.edge_backtrack_ratio", 0.8)
            # 倒车动画未完成：继续后退，不立即 snap
            if agent._backtrack_distance < threshold:
                return  # 保持 BACKTRACK，等下一个 tick 继续倒车

            # 倒车到位：吸附回安全节点 + R1/R2 判定
            agent._snap_to_node(task.from_node)

            # 倒车脱困的入口朝向：车头仍朝原 to_node；用 has_safe_exit 判定后方出口
            entry_heading = agent.yaw_deg
            if has_safe_exit(agent.topo, task.from_node, entry_heading,
                             agent.blocked_edges):
                outs = iter_out_edges(agent.topo, task.from_node,
                                      entry_heading, agent.blocked_edges)
                agent._log_event(
                    "backtrack_r1",
                    f"倒车到 {task.from_node}，{len(outs)} 条可进入出口"
                )
            else:
                agent._log_event(
                    "backtrack_r2",
                    f"倒车转弯到 {task.from_node}，车头翻转驶回已探索方向"
                )
                agent.yaw_deg = edge_heading(agent.topo, task.to_node, task.from_node)
                agent.odom_yaw = agent.yaw_deg
        # 倒车到位：清空 reverse 队列 + 投影 GLOBAL_PLANNING（下一步重规划）
        agent._on_reverse_done()
