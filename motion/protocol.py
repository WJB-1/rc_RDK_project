"""与 test_correct STM32 调试协议一致的最小帧编解码。"""

import struct
from enum import IntEnum

SOF = b"\xC3\x3C"
VERSION = 0x01
MAX_PAYLOAD = 16
PROCESSING_VALUE = -32768


class FrameType(IntEnum):
    HELLO = 0x01
    ACTION = 0x10
    VISION_CORRECTION = 0x11
    HELLO_ACK = 0x81
    ACTION_ACK = 0x90
    ACTION_DONE = 0x91
    REJECT = 0x92


class Action(IntEnum):
    CORRECT = 0x01
    TURN_LEFT = 0x02
    TURN_RIGHT = 0x03
    STRAIGHT = 0x04
    STOP = 0x05


class TurnDirection(IntEnum):
    FORWARD = 0x00
    BACKWARD = 0x01


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def build_frame(frame_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 16 bytes")
    body = bytes((VERSION, int(frame_type), len(payload))) + payload
    return SOF + body + struct.pack("<H", crc16(body))


def encode_action(action: Action, direction: TurnDirection | None = None) -> bytes:
    payload = bytes((int(action),))
    if action in (Action.TURN_LEFT, Action.TURN_RIGHT):
        if direction is None:
            raise ValueError("turn action requires direction")
        payload += bytes((int(direction),))
    return build_frame(FrameType.ACTION, payload)


def encode_straight(distance_mm: int, direction: TurnDirection = TurnDirection.FORWARD) -> bytes:
    if not 1 <= int(distance_mm) <= 0xFFFF:
        raise ValueError("distance must be between 1 and 65535 mm")
    return build_frame(FrameType.ACTION, bytes((Action.STRAIGHT, direction)) + struct.pack("<H", int(distance_mm)))


def encode_turn(action: Action, direction: TurnDirection) -> bytes:
    if action not in (Action.TURN_LEFT, Action.TURN_RIGHT):
        raise ValueError("action must be TURN_LEFT or TURN_RIGHT")
    return encode_action(action, direction)


def encode_vision_correction(offset_mm: float, target_yaw_deg: float) -> bytes:
    offset = max(-200, min(200, int(round(offset_mm))))
    yaw = max(-180.0, min(179.9, float(target_yaw_deg)))
    return build_frame(FrameType.VISION_CORRECTION, struct.pack("<hh", offset, int(round(yaw * 10))))


def encode_vision_processing() -> bytes:
    return build_frame(FrameType.VISION_CORRECTION, struct.pack("<hh", PROCESSING_VALUE, PROCESSING_VALUE))


class FrameDecoder:
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes):
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(SOF)
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(SOF[:1]) else b""
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 7:
                break
            length = self._buffer[4]
            if length > MAX_PAYLOAD:
                del self._buffer[0]
                continue
            total = 7 + length
            if len(self._buffer) < total:
                break
            body = bytes(self._buffer[2:5 + length])
            expected = struct.unpack("<H", self._buffer[5 + length:total])[0]
            if body[0] != VERSION or crc16(body) != expected:
                del self._buffer[0]
                continue
            try:
                frame_type = FrameType(body[1])
            except ValueError:
                frame_type = body[1]
            frames.append((frame_type, body[3:]))
            del self._buffer[:total]
        return frames
