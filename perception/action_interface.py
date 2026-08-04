"""
perception/action_interface.py - 导航大脑 -> 底盘电控 动作下发接口

基于《物理地图建模与Agent接口规范.md》第3.2节实现。

职责：
- 封装 AgentStateMachine.tick() 产出的动作字典
- 提供与下位机通讯协议的序列化/反序列化
- 支持语音播报 (TTS) 调用
"""

import time
from typing import Dict, Optional, Callable

try:
    from ..navigation.state_machine import AgentStateMachine, AgentState
    from ..navigation.map_config import ACTION_TYPES
except ImportError:
    from navigation.state_machine import AgentStateMachine, AgentState
    from navigation.map_config import ACTION_TYPES


class ActionInterface:
    """
    动作下发接口

    作为 AgentStateMachine 与底层电控/STM32 之间的胶水层。
    """

    def __init__(self, agent: AgentStateMachine,
                 send_callback: Callable[[bytes], None] = None):
        """
        Args:
            agent: Agent 状态机实例
            send_callback: 向下位机发送字节流的回调函数（可选）
        """
        self.agent = agent
        self.send_callback = send_callback or self._default_send_callback
        self._last_action: Optional[Dict] = None
        self._tts_queue: list = []
        self._tts_callback: Optional[Callable[[str], None]] = None

    # ============================================================
    # 核心动作下发
    # ============================================================

    def tick(self) -> Optional[Dict]:
        """
        每一控制周期调用，从 Agent 获取动作并下发。

        Returns:
            动作字典（如果状态机要求下发），否则 None。
        """
        action = self.agent.tick()
        if action is not None:
            self._last_action = action
            self._dispatch_action(action)
        return action

    def get_next_action(self) -> Dict:
        """
        显式请求下一个动作（模拟电控在路口中心主动请求的场景）。

        根据规范，当小车距离目标节点逼近路口中心时，电控主动请求，
        或 Agent 主动下发。
        """
        # 强制触发一次状态机 tick（在 APPROACHING 结束后会产出动作）
        action = self.agent.tick()
        if action is None:
            # 如果当前没有待下发动作，返回一个保持指令
            return {
                "action": "STRAIGHT",
                "target_node": self.agent.current_node,
                "expected_yaw": self.agent.yaw_deg,
            }
        self._last_action = action
        self._dispatch_action(action)
        return action

    # ============================================================
    # 语音播报
    # ============================================================

    def execute_voice_broadcast(self, text: str):
        """
        调用外设喇叭播报。

        Args:
            text: 播报文本，例如 "到达3号巡逻点"
        """
        self._tts_queue.append({
            "timestamp": time.time(),
            "text": text,
        })
        if self._tts_callback:
            self._tts_callback(text)

    def set_tts_callback(self, callback: Callable[[str], None]):
        """设置 TTS 回调函数（用于替换真实硬件调用）"""
        self._tts_callback = callback

    # ============================================================
    # 序列化与协议
    # ============================================================

    @staticmethod
    def serialize_action(action: Dict) -> bytes:
        """
        将动作字典序列化为简单的字节协议（示例）。

        协议格式（定长 8 字节，仅作演示）：
            [0xAA] [CMD] [target_idx] [expected_yaw/2] [reserved x4]
        """
        # 映射动作类型到命令字节
        action_map = {
            "TURN_LEFT": 0x01,
            "TURN_RIGHT": 0x02,
            "STRAIGHT": 0x03,
            "STOP": 0x04,
            "UTURN": 0x05,
        }
        cmd = action_map.get(action.get("action", "STOP"), 0x04)

        # 目标节点简单编码（取 N 后面的数字，非数字则 0xFF）
        target = action.get("target_node", "")
        try:
            if target.startswith("N"):
                target_idx = int(target[1:])
            elif target == "START":
                target_idx = 0
            else:
                target_idx = 0xFF
        except ValueError:
            target_idx = 0xFF

        yaw = action.get("expected_yaw", 0.0)
        yaw_byte = int((yaw + 180) / 2) & 0xFF  # 映射 [-180,180] -> [0,180]

        packet = bytes([0xAA, cmd, target_idx & 0xFF, yaw_byte, 0x00, 0x00, 0x00, 0x00])
        return packet

    @staticmethod
    def deserialize_action(packet: bytes) -> Dict:
        """反序列化字节协议为动作字典（调试用）"""
        if len(packet) < 4 or packet[0] != 0xAA:
            raise ValueError("无效的数据包")
        action_map_inv = {
            0x01: "TURN_LEFT",
            0x02: "TURN_RIGHT",
            0x03: "STRAIGHT",
            0x04: "STOP",
            0x05: "UTURN",
        }
        cmd = packet[1]
        target_idx = packet[2]
        yaw_byte = packet[3]

        action_name = action_map_inv.get(cmd, "STOP")
        target_node = f"N{target_idx}" if target_idx != 0xFF and target_idx != 0 else "START"
        expected_yaw = yaw_byte * 2.0 - 180.0

        return {
            "action": action_name,
            "target_node": target_node,
            "expected_yaw": expected_yaw,
        }

    # ============================================================
    # 内部方法
    # ============================================================

    def _dispatch_action(self, action: Dict):
        """实际向下位机发送动作"""
        packet = self.serialize_action(action)
        self.send_callback(packet)

    def _default_send_callback(self, packet: bytes):
        """默认发送回调（仅打印日志，无真实硬件交互）"""
        print(f"[ActionInterface] SEND -> {packet.hex()}")

    def get_last_action(self) -> Optional[Dict]:
        return self._last_action

    def get_tts_queue(self) -> list:
        return self._tts_queue[:]
