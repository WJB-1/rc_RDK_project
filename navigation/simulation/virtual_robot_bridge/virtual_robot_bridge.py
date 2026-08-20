"""
simulation/virtual_robot_bridge/virtual_robot_bridge.py — 虚拟下位机

虚拟下位机 = 真机 STM32 + communication/robot_bridge.py 的替身。

接口形态（2026-08-20 拍板，方案 b）：
  - 独立类，签名对齐 communication/robot_bridge.py 的发送方法，但不继承、不复用真机代码。
  - 控制层（编排层）通过参数选择走哪套 I/O（duck typing），本类只做仿真的底盘运动模拟。

职责：模拟底盘运动（理想化、无误差），但按速度延时（可视化不瞬移）。
  - advance：直行前进，沿当前边理想化插值。
  - backtrack：倒车，沿边反向插值。
  - execute_turn：转弯，瞬时改 yaw 到位。
  中断（定时器延时回调）由 sim_engine 驱动（见 sim_engine._step_internal）。

未接入：send_intersection_turn / send_lane_offset（纠偏理想化不做）。
调用方：当前由 sim_engine 代调（债务 P-23），后续归还给控制层。
"""
import math
from typing import Optional

from ...contracts import OdomUpdate, TurnCommand


class VirtualRobotBridge:
    """虚拟下位机 —— 底盘运动的仿真替身。"""

    def __init__(self, agent, topo):
        self._agent = agent
        self._topo = topo
        self._total_distance = 0.0

    # ================================================================
    # 前进 / 倒车（理想化插值，无误差）
    # ================================================================

    def advance(self, speed_mms: float, dt: float):
        """
        前进：沿当前边推进，注入里程 + 沿边插值定位。

        理想化无误差：直接沿 from_node→to_node 线性插值，不靠 yaw 积分（杜绝漂移）。
        """
        step_mm = speed_mms * dt
        self._total_distance += step_mm
        odom = OdomUpdate(dx_mm=0.0, dy_mm=step_mm, dyaw_deg=0.0, timestamp=0.0)
        self._agent.on_odom_update(odom)
        self._interpolate_position(forward=True)

    def backtrack(self, speed_mms: float, dt: float):
        """倒车：沿当前边反向推进，里程为负让 _backtrack_distance 累积。"""
        step_mm = speed_mms * dt
        self._total_distance += step_mm
        odom = OdomUpdate(dx_mm=0.0, dy_mm=-step_mm, dyaw_deg=0.0, timestamp=0.0)
        self._agent.on_odom_update(odom)
        self._interpolate_position(forward=False)

    def execute_turn(self, cmd: TurnCommand):
        """转弯：瞬时改 yaw 到位（理想化，无角速度过程）。"""
        self._agent.yaw_deg = cmd.expected_yaw
        self._agent.odom_yaw = cmd.expected_yaw
        self._agent._log_event(
            "turn_execute", f"转向 {cmd.action.value} → yaw={cmd.expected_yaw:.1f}"
        )

    # ================================================================
    # 位置插值（沿边理想化定位）
    # ================================================================

    def _interpolate_position(self, forward: bool):
        """沿当前边线性插值，定位 agent 到边上的精确点（无漂移）。"""
        task = self._agent.executor.current_task
        if task is None:
            return
        try:
            a = self._topo.get_node(task.from_node)
            b = self._topo.get_node(task.to_node)
        except KeyError:
            return
        edge_len = task.distance_mm
        if edge_len <= 0:
            return

        if forward:
            car_pos = self._agent._cumulative_odom - self._agent.executor._start_odom
            ratio = max(0.0, min(1.0, car_pos / edge_len))
            x = a.x_mm + (b.x_mm - a.x_mm) * ratio
            y = a.y_mm + (b.y_mm - a.y_mm) * ratio
        else:
            # 倒车：从 to_node 往 from_node 退，比例用 _backtrack_distance
            ratio = max(0.0, min(1.0, self._agent._backtrack_distance / edge_len))
            x = b.x_mm + (a.x_mm - b.x_mm) * ratio
            y = b.y_mm + (a.y_mm - b.y_mm) * ratio

        self._agent.x_mm = x
        self._agent.y_mm = y
        # yaw 对齐边方向（前进沿 from→to，倒车保持朝 to_node）
        dx = b.x_mm - a.x_mm
        dy = b.y_mm - a.y_mm
        self._agent.yaw_deg = math.degrees(math.atan2(dx, dy))
        while self._agent.yaw_deg > 180:
            self._agent.yaw_deg -= 360
        while self._agent.yaw_deg < -180:
            self._agent.yaw_deg += 360

    @property
    def total_distance(self) -> float:
        return self._total_distance
