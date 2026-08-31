"""
simulation/virtual_task_process/task_process.py — 虚拟任务处理

响应仿真里的任务事件：RFID 打卡、涵洞探索（侦查）的任务发起。

接口（2026-08-20 拍板）：简单，状态就「执行成功 / 失败」两种。
后期加概率失败逻辑，测试 D-12 冷却重试（债务 D-17）。

调用方：当前由 sim_engine 代调（债务 P-24），后续归还给控制层。
"""
from typing import Optional

from ...contracts import RfidEvent


class VirtualTaskProcess:
    """虚拟任务处理 —— 响应 RFID 打卡 / 涵洞探索的任务发起。"""

    def __init__(self, agent, topo, scene):
        self._agent = agent
        self._topo = topo
        self._scene = scene

    def respond_rfid(self, arrival_node: str, sim_time: float) -> bool:
        """
        响应 RFID 打卡：若到达节点有 RFID，发 RfidEvent 给 agent。

        返回 bool：True=成功打卡，False=该节点无 RFID（不响应）。
        """
        if arrival_node not in self._topo.nodes:
            return False
        topo_node = self._topo.get_node(arrival_node)
        if not topo_node.has_rfid:
            return False
        event = RfidEvent(
            uid=arrival_node,
            node_name=arrival_node,
            timestamp=sim_time,
        )
        self._agent.on_rfid_scanned(event)
        return True

    def respond_culvert_explore(self, arrival_node: str) -> None:
        """
        响应涵洞探索任务发起。

        TODO(D-16)：涵洞侦查的「发起/验收」真正的归属是控制层（债务 D-16 记）。
        当前这里只是占位——真正的发起/验收逻辑由控制层实现，虚拟任务处理只
        模拟「响应」这件事。后续概率失败（D-17）也挂在这里。
        """
        # 占位：涵洞探索的任务发起响应，后续控制层实现 D-16 后填充。
        pass
