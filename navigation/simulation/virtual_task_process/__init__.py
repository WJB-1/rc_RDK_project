"""
simulation/virtual_task_process/ — 虚拟任务处理

响应任务事件（RFID 打卡、涵洞探索发起）。接口简单：成功/失败两态。
当前由 sim_engine 代调（债务 P-24），后续归还给控制层。
"""
from .task_process import VirtualTaskProcess

__all__ = ["VirtualTaskProcess"]
