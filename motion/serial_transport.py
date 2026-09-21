"""UART 独占传输适配器，隐藏串口实现并提供字节流回调。"""

import threading
import time


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

    def subscribe(self, listener):
        self._listener = listener

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

    def feed(self, data):
        if self._listener is not None:
            self._listener(data)

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
