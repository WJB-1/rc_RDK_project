"""
perception/perception_adapter.py - 感知适配层 (Perception Adapter)

基于《物理地图建模与Agent接口规范.md》第3.1节实现。
职责：将底盘/视觉节点的原始事件转换为对 AgentStateMachine 的标准化调用。

支持事件：
- 里程计更新 (on_odom_update)
- 视觉路口检测 (on_crossroad_detected)
- RFID 打卡 (on_rfid_scanned)
- 涵洞检测 (on_culvert_detected)
"""

import time
from typing import Optional, Dict

from robocup_rescue_brain.navigation.state_machine import AgentStateMachine
from robocup_rescue_brain.navigation.contracts import (
    OdomUpdate, RfidEvent, CrossroadEvent,
)


class PerceptionAdapter:
    """
    感知适配器

    作为底盘/视觉节点与 Agent 之间的胶水层：
    - 对输入数据进行单位换算、合法性校验与数据类包装
    - 调用 AgentStateMachine 的对应事件处理函数
    - 提供事件节流/防抖（可选，后续可扩展）
    """

    def __init__(self, agent: AgentStateMachine):
        self.agent = agent
        self._last_odom_time = 0.0
        self._crossing_cooldown = 0.0  # 路口检测冷却 (s)
        self._last_crossing_time = 0.0

    # ============================================================
    # 标准接口
    # ============================================================

    def on_odom_update(self, dx_mm: float, dy_mm: float, dyaw_deg: float,
                       timestamp: float = None):
        """
        底盘轮速计或 IMU 更新。

        Args:
            dx_mm: 车体坐标系下 X 方向位移 (mm)
            dy_mm: 车体坐标系下 Y 方向位移 (mm)
            dyaw_deg: 航向角变化 (deg)
            timestamp: 事件时间戳（可选，默认使用系统时间）
        """
        if timestamp is not None:
            self._last_odom_time = timestamp
        self.agent.on_odom_update(OdomUpdate(
            dx_mm=dx_mm, dy_mm=dy_mm, dyaw_deg=dyaw_deg,
            timestamp=timestamp or time.time(),
        ))

    def on_crossroad_detected(self, distance_mm: float,
                              duty_cycle: float = 1.0,
                              timestamp: float = None) -> bool:
        """
        视觉算法检测到前方存在横向路口边缘线。

        Args:
            distance_mm: 到路口横向线的距离 (mm)
            duty_cycle: 检测占空比
            timestamp: 事件时间戳

        Returns:
            bool: 事件是否被成功处理（冷却期内会忽略）
        """
        now = timestamp or time.time()
        if now - self._last_crossing_time < self._crossing_cooldown:
            return False
        self._last_crossing_time = now
        self.agent.on_crossroad_detected(CrossroadEvent(
            distance_mm=distance_mm, duty_cycle=duty_cycle, timestamp=now,
        ))
        return True

    # 向后兼容旧调用方 (on_vision_crossing_detected)
    def on_vision_crossing_detected(self, distance_mm: float,
                                    timestamp: float = None) -> bool:
        return self.on_crossroad_detected(distance_mm, timestamp=timestamp)

    def on_rfid_scanned(self, uid: str):
        """
        RFID 读卡器扫到任务点标签。

        Args:
            uid: 标签 UID，对应节点名称（如 "N1", "N2"）
        """
        uid_clean = uid.strip().upper()
        self.agent.on_rfid_scanned(RfidEvent(uid=uid_clean, timestamp=time.time()))

    def on_culvert_detected(self, local_x_mm: float, local_y_mm: float):
        """
        视觉网络检测到涵洞入口。

        Args:
            local_x_mm: 涵洞相对于车体的局部 X 坐标 (mm)
            local_y_mm: 涵洞相对于车体的局部 Y 坐标 (mm)
        """
        self.agent.on_culvert_detected(local_x_mm, local_y_mm)

    # ============================================================
    # 扩展接口（便于测试与调试）
    # ============================================================

    def set_crossing_cooldown(self, seconds: float):
        """设置路口检测冷却时间 (s)，防止同一路口重复触发"""
        self._crossing_cooldown = max(0.0, seconds)

    def get_agent_position(self) -> tuple:
        """透传获取 Agent 当前物理坐标"""
        return self.agent.get_position()

    def get_agent_state(self) -> str:
        """透传获取 Agent 当前状态名"""
        return self.agent.get_state_name()
