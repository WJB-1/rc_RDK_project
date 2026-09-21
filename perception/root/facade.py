"""视觉大层的唯一公开门面。"""

import time
import threading

from .contracts import LaneControlSample, LaneSampleStatus, VisionHealthState
from .lane_assist import LaneAssistService
from .runtime import CameraRuntime


class PerceptionFacade:
    def __init__(self, frame_source, pipeline, clock_ns=None, poll_interval_s=0.05):
        self._clock_ns = clock_ns or time.monotonic_ns
        self._runtime = CameraRuntime(frame_source, self._clock_ns)
        self._pipeline = pipeline
        self._lane_assist = LaneAssistService()
        self._health = VisionHealthState.STOPPED
        self._poll_interval_s = poll_interval_s
        self._stop_event = threading.Event()
        self._thread = None

    @property
    def lane_assist(self):
        return self._lane_assist

    @property
    def health(self):
        return self._health

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._health = VisionHealthState.READY

    def stop(self):
        self._stop_event.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        session_id = self._lane_assist.active_session_id
        if session_id is not None:
            self._lane_assist.stop(session_id)
        self._health = VisionHealthState.STOPPED

    def publish_once(self):
        session_id = self._lane_assist.active_session_id
        if session_id is None:
            return
        frame_id, captured_at, frame = self._runtime.next_front_frame()
        published_at = self._clock_ns()
        if frame is None:
            sample = LaneControlSample.invalid(
                session_id, frame_id, captured_at, published_at,
                LaneSampleStatus.PROCESSING, "front camera frame unavailable",
            )
        else:
            analysis = self._pipeline.analyze(frame, frame_id, captured_at)
            measurement = analysis.lane_measurement
            if measurement.status is LaneSampleStatus.VALID:
                sample = LaneControlSample.valid(
                    session_id, frame_id, captured_at, published_at,
                    measurement.quality_score, measurement.offset_mm, measurement.target_yaw_deg,
                )
            else:
                sample = LaneControlSample.invalid(
                    session_id, frame_id, captured_at, published_at,
                    measurement.status, measurement.reason or "lane analysis unavailable",
                    measurement.quality_score,
                )
        self._lane_assist.publish(sample)

    def _run(self):
        while not self._stop_event.is_set():
            self.publish_once()
            self._stop_event.wait(self._poll_interval_s)
