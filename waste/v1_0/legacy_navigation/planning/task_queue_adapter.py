"""
planning/task_queue_adapter.py — 任务队列适配器（TaskQueueAdapter）

控制层/编排层的薄封装，把散在 agent 字段里的任务状态，转译成
JunctionDecider 需要的 task_queue 接口（pending_rfid / pending_culverts）。

职责边界：
  - 路径规划（JunctionDecider）只依赖 task_queue 接口，不读 agent 内部。
  - 本适配器由控制层实例化，负责「从 agent/topo/runtime_map 提炼任务状态」。

「顺路打卡」债务：当前控制层尚未实现「路径路过未打卡 RFID 就顺手打卡」，
D-03 现场 Dijkstra 允许穿过未打卡 center（这是特性），计为后续债务待闭环。
"""
from typing import List


class TaskQueueAdapter:
    """把 agent 的散状任务状态转译成 task_queue 接口。"""

    def __init__(self, agent):
        self._agent = agent

    def pending_rfid(self) -> List[str]:
        """未打卡的 RFID 任务点（方块中心节点名）。"""
        return [n for n in self._agent.topo.nodes
                if self._agent.topo.nodes[n].has_rfid
                and not self._agent.topo.nodes[n].is_visited]

    def pending_culverts(self) -> List[int]:
        """
        未侦查的涵洞边（edge_id）。

        分档逻辑与 orchestrator.do_global_planning 一致：
        - 还有未打卡 RFID → 只排「已发现未侦查」的涵洞边（车已知位置，顺路侦查）。
        - RFID 全完成 → 扫荡兜底：用场景真值把所有未侦查涵洞边都排上。
        """
        agent = self._agent
        discovered_unrecon = agent.discovered_culverts - agent.recon_culverts
        if self.pending_rfid():
            return sorted(discovered_unrecon)
        return sorted(agent._culvert_targets - agent.recon_culverts)
