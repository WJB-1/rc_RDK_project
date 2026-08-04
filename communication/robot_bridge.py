"""
communication/robot_bridge.py - 机器人主桥接层

连接 STM32 下位机 <-> navigation Agent 状态机

核心数据流 (50Hz闭环):

  STM32(UART) -> decode_frame -> OdomIntegrator -> Agent.on_odom_update
                                                    v
  Agent.tick() -> action -> encode_velocity/turn -> STM32(UART)

  外部轮询: bridge.get_position() -> (x_mm, y_mm, yaw_deg)
"""

import time
import logging
import threading
from typing import Optional, Tuple, Callable

from .stm32_protocol import (
    FbID, StmStatusCode, CmdID, ActionCode,
    decode_frame, decode_odom, decode_status, decode_sensor_uid,
    encode_velocity, encode_turn, encode_action, encode_intersection_turn,
    encode_lane_offset,
)

logger = logging.getLogger("robot_bridge")


class OdomIntegrator:
    """
    里程计积分器 — 将 STM32 反馈转为 (dx, dy, dyaw)

    STM32 每帧上报: (总距离, 当前航向角, 当前速度)
    积分器用差分法计算每帧的位移增量。

    关键设计:
    - 距离用差值 (delta_dist) -> 前进方向位移
    - Yaw 用 IMU 直接值 (STM32已做融合)
    - 转弯期间: 距离增量~0, yaw 变化由 agent.on_odom_update 处理
    """

    def __init__(self):
        self._last_dist: Optional[float] = None
        self._last_yaw: float = 0.0
        self._pending_turn_yaw: Optional[float] = None

    def reset(self):
        """里程计归零 (对应 CMD_RESET_ODOM 后调用)"""
        self._last_dist = None
        self._pending_turn_yaw = None

    def on_turn_cmd(self, angle_deg: float):
        """记录转弯目标角度"""
        self._pending_turn_yaw = angle_deg

    def update(self, dist_mm: int, yaw_deg: float, speed_mms: int,
               stm_status: Optional[int] = None) -> Tuple[float, float, float]:
        """
        单步积分。

        Args:
            dist_mm: STM32累计距离 (自上次归零)
            yaw_deg: STM32当前航向 (IMU+编码器融合)
            speed_mms: 当前速度
            stm_status: 下位机状态码

        Returns:
            (dx_mm, dy_mm, dyaw_deg) 车体坐标系下的位移
        """
        dx, dy, dyaw = 0.0, 0.0, 0.0

        if self._last_dist is not None:
            delta_dist = dist_mm - self._last_dist
            if delta_dist > 0:
                dy = delta_dist  # 前进方向
        else:
            # 首次收到，仅记录基线
            self._last_yaw = yaw_deg

        # Yaw 更新: 直接使用 STM32 融合后的航向角变化
        if self._last_dist is not None:
            dyaw = yaw_deg - self._last_yaw
            # 归一化到 [-180, 180]
            while dyaw > 180:
                dyaw -= 360
            while dyaw < -180:
                dyaw += 360

        # 转弯完成: 确保角度跳变被正确记录
        if stm_status == StmStatusCode.TURN_DONE and self._pending_turn_yaw is not None:
            dyaw = self._pending_turn_yaw
            self._pending_turn_yaw = None

        self._last_dist = float(dist_mm)
        self._last_yaw = yaw_deg

        return (dx, dy, dyaw)


class RobotBridge:
    """
    机器人桥接器 — 主循环 (50Hz)

    使用:
        bridge = RobotBridge()
        bridge.start()  # 后台线程运行

        # 串口回调中调用:
        bridge.on_stm32_data(raw_bytes)

        # 外部轮询:
        x, y, yaw = bridge.get_position()
        summary = bridge.get_status_summary()

        bridge.stop()
    """

    def __init__(self, agent=None, serial_send: Optional[Callable[[bytes], None]] = None):
        try:
            from ..navigation.state_machine import AgentStateMachine
            from ..perception.perception_adapter import PerceptionAdapter
        except ImportError:
            from navigation.state_machine import AgentStateMachine
            from perception.perception_adapter import PerceptionAdapter

        self.agent = agent or AgentStateMachine()
        self.adapter = PerceptionAdapter(self.agent) if agent else None

        self.odom = OdomIntegrator()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # 串口
        self._serial_send = serial_send or self._default_send
        self._rx_buffer = bytearray()
        self._last_stm_status: Optional[int] = None

        # 50Hz
        self._hz = 50
        self._period_s = 1.0 / self._hz

        # 统计
        self._tick_count = 0
        self._odom_count = 0
        self._last_pos_sent = 0.0

    def set_agent(self, agent):
        try:
            from ..perception.perception_adapter import PerceptionAdapter
        except ImportError:
            from perception.perception_adapter import PerceptionAdapter
        self.agent = agent
        self.adapter = PerceptionAdapter(agent)

    def set_serial_send(self, func: Callable[[bytes], None]):
        self._serial_send = func

    # ============================================================
    # STM32 数据入口 (串口接收线程调用)
    # ============================================================

    def on_stm32_data(self, data: bytes):
        """STM32 原始字节流入口，自动处理粘包"""
        self._rx_buffer.extend(data)
        while True:
            result = decode_frame(bytes(self._rx_buffer))
            if result is None:
                break
            fb_type, payload, frame_total = result
            self._rx_buffer = self._rx_buffer[frame_total:]
            self._handle_frame(fb_type, payload)

    def _handle_frame(self, fb_type: int, payload: bytes):
        """处理解析后的反馈帧"""
        if fb_type == FbID.ODOMETRY:
            self._odom_count += 1
            parsed = decode_odom(payload)
            if parsed is not None:
                dist_mm, yaw_deg, speed_mms = parsed
                dx, dy, dyaw = self.odom.update(
                    dist_mm, yaw_deg, speed_mms, self._last_stm_status
                )
                if self.adapter and (abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dyaw) > 0.001):
                    self.adapter.on_odom_update(dx, dy, dyaw)

        elif fb_type == FbID.STATUS:
            status = decode_status(payload)
            if status is not None:
                self._last_stm_status = status
                try:
                    logger.info(f"STM32 status: {StmStatusCode(status).name}")
                except ValueError:
                    logger.info(f"STM32 status: {status}")

        elif fb_type == FbID.SENSOR:
            uid = decode_sensor_uid(payload)
            if uid and self.adapter:
                self.adapter.on_rfid_scanned(uid)

    # ============================================================
    # 主循环
    # ============================================================

    def start(self):
        """后台启动 50Hz 主循环"""
        if self._running:
            return
        self._running = True

        if self.agent and hasattr(self.agent, 'start'):
            self.agent.start()

        self._thread = threading.Thread(target=self._main_loop, daemon=True)
        self._thread.start()
        logger.info(f"RobotBridge started @ {self._hz}Hz")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        logger.info(f"RobotBridge stopped (odom_frames={self._odom_count}, ticks={self._tick_count})")

    def _main_loop(self):
        while self._running:
            t0 = time.time()
            try:
                self._tick()
            except Exception as e:
                logger.error(f"Tick error: {e}")
            elapsed = time.time() - t0
            remaining = self._period_s - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _tick(self):
        """单次 tick: Agent -> 编解码 -> STM32"""
        if not self.agent:
            return

        action = self.agent.tick()
        if action is None:
            return

        self._tick_count += 1
        packets = self._action_to_stm32(action)
        for pkt in packets:
            self._serial_send(pkt)

    def _action_to_stm32(self, action) -> list:
        """
        Agent 动作 -> STM32 指令列表。
        返回 list[bytes] 因为一个动作可能拆成多条指令。

        兼容两种 action 格式:
          - TurnCommand (来自 agent.tick())
          - dict {"action": "STOP"/...} (来自 Web 面板手动控制或旧代码)

        注意: 下位机正角度表示左转
        """
        # 兼容 TurnCommand (dataclass) 和 dict
        if hasattr(action, 'action'):
            # TurnCommand: action 是 TurnAction 枚举
            atype = action.action.value  # "straight", "turn_left", etc.
            atype = atype.upper()        # "STRAIGHT", "TURN_LEFT", etc.
        else:
            atype = action.get("action", "STOP")

        if atype in ("TURN_LEFT", "turn_left"):
            pkts = [encode_action(ActionCode.STOP), encode_turn(90)]
            self.odom.on_turn_cmd(90)
            return pkts

        elif atype in ("TURN_RIGHT", "turn_right"):
            pkts = [encode_action(ActionCode.STOP), encode_turn(-90)]
            self.odom.on_turn_cmd(-90)
            return pkts

        elif atype in ("UTURN", "uturn"):
            pkts = [encode_action(ActionCode.STOP), encode_turn(180)]
            self.odom.on_turn_cmd(180)
            return pkts

        elif atype in ("STRAIGHT", "straight"):
            return [encode_velocity(300, 0)]

        elif atype in ("STOP", "stop"):
            return [encode_action(ActionCode.STOP)]

        return [encode_action(ActionCode.STOP)]

    def send_turn_command(self, cmd) -> None:
        """
        适配新架构的 TurnCommand。
        :param cmd: navigation.contracts.TurnCommand
        """
        pkts = self._action_to_stm32(cmd)
        for pkt in pkts:
            self._serial_send(pkt)

    def send_velocity(self, vx_mms: float, wz_mrads: float):
        """外部直接发送速度指令 (用于视觉伺服)"""
        pkt = encode_velocity(vx_mms, wz_mrads)
        logger.info(f"[SEND VELOCITY] vx={vx_mms}, wz={wz_mrads}, pkt={pkt.hex()}")
        self._serial_send(pkt)

    def send_turn_imu(self, angle_deg: float):
        """发送原地 IMU 转向指令 (CMD_TURN_IMU 0x03)"""
        logger.info(f"[SEND TURN_IMU] angle={angle_deg}")
        self._serial_send(encode_turn(angle_deg))
        self.odom.on_turn_cmd(angle_deg)

    def send_intersection_turn(self, direction: str):
        """发送路口边走边转指令 (CMD_INTERSECTION_TURN 0x05)"""
        logger.info(f"[SEND INTERSECTION_TURN] direction={direction}")
        self._serial_send(encode_intersection_turn(direction))

    def send_lane_offset(self, offset_mm: float):
        """发送车道横向偏移给下位机 (CMD_LANE_OFFSET 0x06)"""
        pkt = encode_lane_offset(offset_mm)
        self._serial_send(pkt)

    def send_reset_odom(self):
        """发送里程清零指令"""
        self._serial_send(encode_action(ActionCode.RESET_ODOM))

    def _default_send(self, data: bytes):
        logger.info(f"[DEFAULT SEND] {data.hex()}")

    # ============================================================
    # 位置查询接口
    # ============================================================

    def get_position(self) -> Tuple[float, float, float]:
        """实时位置 (x_mm, y_mm, yaw_deg)"""
        if self.agent and hasattr(self.agent, 'get_position'):
            return self.agent.get_position()
        return (0.0, 0.0, 0.0)

    def get_state(self) -> str:
        if self.agent and hasattr(self.agent, 'get_state_name'):
            return self.agent.get_state_name()
        return "UNKNOWN"

    def get_visited(self) -> list:
        if hasattr(self.agent, 'visited_nodes'):
            return list(self.agent.visited_nodes)
        return []

    def get_status_summary(self) -> dict:
        x, y, yaw = self.get_position()
        return {
            "state": self.get_state(),
            "x_mm": round(x, 1),
            "y_mm": round(y, 1),
            "yaw_deg": round(yaw, 1),
            "current_node": getattr(self.agent, 'current_node', None),
            "target_node": getattr(self.agent, 'target_node', None),
            "visited": self.get_visited(),
            "tick_count": self._tick_count,
            "odom_frames": self._odom_count,
        }
