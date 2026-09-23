import math
import threading

from perception.contracts import FrameAnalysis, LaneMeasurement, LaneSampleStatus
from perception.facade import PerceptionFacade


class VisionDebugPipeline:
    def __init__(self, tracker):
        self._tracker = tracker
        self._lock = threading.Lock()
        self._views = {}
        self._snapshot = {"frame_id": 0, "views": [], "lane": {}, "timing_ms": {}, "metrics": {}}

    def analyze(self, frame, frame_id, captured_at_monotonic_ns):
        offset_mm, is_intersection, debug_frame = self._tracker.process(frame)
        lane_state = getattr(self._tracker, "last_lane_state", None) or {}
        if lane_state.get("frame_dropped", False):
            measurement = LaneMeasurement(
                LaneSampleStatus.NO_VALID_LANE, None, None,
                float(lane_state.get("quality_score", 0.0)),
                lane_state.get("drop_reason") or "lane tracker dropped frame",
            )
        else:
            measurement = LaneMeasurement(
                LaneSampleStatus.VALID,
                float(offset_mm),
                math.degrees(float(lane_state.get("lane_angle_rad", 0.0))),
                float(lane_state.get("quality_score", 1.0)),
            )
        capture = getattr(self._tracker, "last_debug_capture", None) or {}
        views = {
            "raw": frame,
            "overlay": debug_frame,
            "original": getattr(self._tracker, "last_original_view", None),
            "segmentation": getattr(self._tracker, "last_seg_mask", None),
            "bev": getattr(self._tracker, "last_bev_mask", None),
        }
        with self._lock:
            self._views = {name: image.copy() for name, image in views.items() if image is not None}
            self._snapshot = {
                "frame_id": int(frame_id),
                "views": list(self._views),
                "lane": {
                    "offset_mm": float(offset_mm),
                    "target_yaw_deg": measurement.target_yaw_deg,
                    "quality_score": measurement.quality_score,
                    "intersection": bool(is_intersection),
                    "status": measurement.status.value,
                },
                "timing_ms": dict(getattr(self._tracker, "last_timing", None) or {}),
                "metrics": dict(capture.get("metrics") or {}),
            }
        return FrameAnalysis(
            int(frame_id), int(captured_at_monotonic_ns), measurement,
            bool(is_intersection), lane_state.get("distance_to_crossroad_mm"),
        )

    def snapshot(self):
        with self._lock:
            result = dict(self._snapshot)
            result["lane"] = dict(result["lane"])
            result["timing_ms"] = dict(result["timing_ms"])
            result["metrics"] = dict(result["metrics"])
            result["views"] = list(result["views"])
            return result

    def jpeg(self, view_name):
        import cv2
        with self._lock:
            image = self._views.get(view_name)
            if image is None:
                return None
            image = image.copy()
        ok, encoded = cv2.imencode(".jpg", image)
        return encoded.tobytes() if ok else None


class VisionDebugRuntime:
    def __init__(self, settings):
        from perception.algorithms.lane.tracker import LaneTracker
        from perception.devices.camera import CameraManager

        camera = CameraManager(settings)
        if not camera.initialize():
            raise RuntimeError("front camera initialization failed")
        try:
            tracker = LaneTracker(settings)
            tracker.set_debug_capture_enabled(True)
        except Exception:
            camera.release()
            raise
        self._camera = camera
        self._pipeline = VisionDebugPipeline(tracker)
        self._perception = PerceptionFacade(camera, self._pipeline)

    @property
    def lane_assist(self):
        return self._perception.lane_assist

    def start(self):
        self._perception.start()

    def stop(self):
        self._perception.stop()
        self._camera.release()

    def snapshot(self):
        return self._pipeline.snapshot()

    def jpeg(self, view_name):
        return self._pipeline.jpeg(view_name)
