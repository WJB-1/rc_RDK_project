"""UART 独占传输适配器，隐藏串口实现并提供字节流回调。"""

import threading
import time
import struct
from collections import deque

from .protocol import Action, FrameDecoder, FrameType, PROCESSING_VALUE, TurnDirection, build_frame


class SerialTransport:
    def __init__(self, port, baudrate=115200, serial_factory=None):
        self.port = port
        self.baudrate = baudrate
        self._serial_factory = serial_factory
        self._serial = None
        self._lock = threading.Lock()
        self._listener = None
        self._stop_event = threading.Event()
        self._reader = None
        self._events = deque(maxlen=300)
        self._event_lock = threading.Lock()
        self._event_sequence = 0
        self._context_provider = None
        self._log_decoders = {"TX": FrameDecoder(), "RX": FrameDecoder()}

    def subscribe(self, listener):
        self._listener = listener

    def set_context_provider(self, provider):
        self._context_provider = provider

    def events(self):
        with self._event_lock:
            return tuple(dict(event) for event in self._events)

    def start(self):
        if self._serial is not None:
            return
        factory = self._serial_factory
        if factory is None:
            import serial
            factory = serial.Serial
        self._serial = factory(self.port, self.baudrate, timeout=0.02, write_timeout=0.2)
        self._stop_event.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def stop(self):
        self._stop_event.set()
        if self._reader is not None and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self._serial is not None and getattr(self._serial, "is_open", True):
            self._serial.close()
        self._serial = None

    def send(self, frame):
        if self._serial is None:
            raise RuntimeError("serial transport is not started")
        with self._lock:
            self._serial.write(frame)
            if hasattr(self._serial, "flush"):
                self._serial.flush()
        self._record_frames("TX", frame)

    def feed(self, data):
        self._record_frames("RX", data)
        if self._listener is not None:
            self._listener(data)

    def _record_frames(self, direction, data):
        for frame_type, payload in self._log_decoders[direction].feed(data):
            try:
                category, name, summary, fields = self._describe(frame_type, payload)
            except Exception as error:
                category, name, summary, fields = "protocol", "DECODE_ERROR", "协议日志解析失败", {"error": str(error)}
            try:
                context = self._context_provider() if self._context_provider is not None else {}
            except Exception:
                context = {}
            with self._event_lock:
                self._event_sequence += 1
                self._events.append({
                    "sequence": self._event_sequence,
                    "timestamp": time.time(),
                    "direction": direction,
                    "category": category,
                    "name": name,
                    "summary": summary,
                    "fields": fields,
                    "hex": build_frame(frame_type, payload).hex(" "),
                    "step": dict(context or {}),
                })

    @staticmethod
    def _describe(frame_type, payload):
        name = frame_type.name if isinstance(frame_type, FrameType) else "TYPE_{:02X}".format(frame_type)
        if frame_type is FrameType.HELLO:
            return "handshake", name, "上位机请求建立运动链路", {}
        if frame_type is FrameType.HELLO_ACK:
            return "handshake", name, "STM32 已确认运动链路", {}
        if frame_type is FrameType.VISION_CORRECTION and len(payload) >= 4:
            offset_mm, yaw_tenths = struct.unpack("<hh", payload[:4])
            if offset_mm == PROCESSING_VALUE and yaw_tenths == PROCESSING_VALUE:
                return "vision", name, "视觉仍在处理，本周期不更新纠偏量", {"status": "processing"}
            fields = {"offset_mm": offset_mm, "target_yaw_deg": yaw_tenths / 10.0}
            return "vision", name, "下发视觉纠偏：横向 {} mm，目标偏航 {:.1f}°".format(offset_mm, yaw_tenths / 10.0), fields
        if frame_type is FrameType.ACTION and payload:
            action = Action(payload[0]) if payload[0] in set(item.value for item in Action) else payload[0]
            action_name = action.name if isinstance(action, Action) else str(action)
            fields = {"action": action_name}
            if action is Action.STRAIGHT and len(payload) >= 4:
                direction = TurnDirection(payload[1]).name
                distance_mm = struct.unpack("<H", payload[2:4])[0]
                fields.update({"direction": direction, "distance_mm": distance_mm})
                return "motion", name, "下发直行：{}，{} mm".format(direction, distance_mm), fields
            if len(payload) >= 2:
                fields["direction"] = TurnDirection(payload[1]).name
            return "motion", name, "下发动作：{}".format(action_name), fields
        if frame_type in (FrameType.ACTION_ACK, FrameType.ACTION_DONE, FrameType.REJECT):
            action_name = Action(payload[0]).name if payload and payload[0] in set(item.value for item in Action) else "UNKNOWN"
            fields = {"action": action_name}
            if frame_type is FrameType.ACTION_ACK:
                return "motion", name, "STM32 已受理动作：{}".format(action_name), fields
            result = payload[1] if len(payload) > 1 else None
            fields["result_code"] = result
            if frame_type is FrameType.ACTION_DONE:
                return "motion", name, "STM32 动作结束：{}，结果码 {}".format(action_name, result), fields
            return "error", name, "STM32 拒绝动作：{}，原因码 {}".format(action_name, result), fields
        return "protocol", name, "收到协议帧 {}".format(name), {"payload_hex": payload.hex(" ")}

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    serial_port = self._serial
                    if serial_port is None:
                        return
                    waiting = getattr(serial_port, "in_waiting", 0)
                    data = serial_port.read(waiting or 1)
                if data:
                    self.feed(data)
            except Exception:
                return
            time.sleep(0.001)
