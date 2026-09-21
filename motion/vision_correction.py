"""将 Perception 回正样本适配为 Motion 的底盘转发回调。"""


class VisionCorrectionAdapter:
    """Motion 不读取相机或运行算法，只订阅视觉门面。"""

    def __init__(self, lane_assist_port):
        self._lane_assist_port = lane_assist_port
        self._session_id = None
        self._callback = None

    def start(self, session_id, callback):
        if self._session_id is not None:
            raise RuntimeError("a correction session is already active")
        self._session_id = session_id
        self._callback = callback
        self._lane_assist_port.start(session_id, self._on_sample)

    def stop(self, session_id):
        if session_id != self._session_id:
            return
        self._lane_assist_port.stop(session_id)
        self._session_id = None
        self._callback = None

    def _on_sample(self, sample):
        if sample.session_id != self._session_id or self._callback is None:
            return
        if sample.lateral_offset_mm is None or sample.target_yaw_deg is None:
            self._callback(sample.session_id, None, None)
            return
        self._callback(sample.session_id, sample.lateral_offset_mm, sample.target_yaw_deg)
