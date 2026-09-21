"""Motion 内部的底盘语义发送边界。"""

from .protocol import (
    Action,
    TurnDirection,
    encode_action,
    encode_straight,
    encode_turn,
    encode_vision_correction,
    encode_vision_processing,
    build_frame,
    FrameType,
)


class ChassisDriver:
    """将动作语义编码为 STM32 帧并交给唯一传输层。"""

    def __init__(self, transport):
        self._transport = transport

    def send_action(self, action, direction=None):
        self._transport.send(encode_action(action, direction))

    def send_hello(self):
        self._transport.send(build_frame(FrameType.HELLO))

    def send_straight(self, distance_mm, direction=TurnDirection.FORWARD):
        self._transport.send(encode_straight(distance_mm, direction))

    def send_turn(self, action, direction):
        self._transport.send(encode_turn(action, direction))

    def send_vision_correction(self, offset_mm, target_yaw_deg):
        self._transport.send(encode_vision_correction(offset_mm, target_yaw_deg))

    def send_vision_processing(self):
        self._transport.send(encode_vision_processing())

    def send_stop(self):
        self.send_action(Action.STOP)
