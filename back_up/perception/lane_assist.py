"""回正会话的最新视觉样本发布器。"""

import threading


class LaneAssistService:
    def __init__(self):
        self._lock = threading.Lock()
        self._session_id = None
        self._consumer = None

    def start(self, session_id, consumer):
        with self._lock:
            if self._session_id is not None:
                raise RuntimeError("a correction session is already active")
            self._session_id = session_id
            self._consumer = consumer

    def stop(self, session_id):
        with self._lock:
            if self._session_id != session_id:
                return
            self._session_id = None
            self._consumer = None

    def publish(self, sample):
        with self._lock:
            if sample.session_id != self._session_id:
                return
            consumer = self._consumer
        if consumer is not None:
            consumer(sample)

    @property
    def active_session_id(self):
        with self._lock:
            return self._session_id
