"""把既有车道跟踪器输出适配为 Perception 门面的结构化结果。"""

import math

from perception.contracts import (
    FrameAnalysis,
    LaneMeasurement,
    LaneSampleStatus,
)


class VisionPipeline:
    def __init__(self, tracker):
        self._tracker = tracker

    def analyze(self, frame, frame_id, captured_at_monotonic_ns):
        offset_mm, is_intersection, _ = self._tracker.process(frame)
        lane_state = getattr(self._tracker, "last_lane_state", None) or {}
        if lane_state.get("frame_dropped", False):
            measurement = LaneMeasurement(
                LaneSampleStatus.NO_VALID_LANE,
                None,
                None,
                float(lane_state.get("quality_score", 0.0)),
                lane_state.get("drop_reason") or "lane tracker dropped frame",
            )
        else:
            angle_rad = float(lane_state.get("lane_angle_rad", 0.0))
            measurement = LaneMeasurement(
                LaneSampleStatus.VALID,
                float(offset_mm),
                math.degrees(angle_rad),
                float(lane_state.get("quality_score", 1.0)),
            )
        return FrameAnalysis(
            int(frame_id),
            int(captured_at_monotonic_ns),
            measurement,
            bool(is_intersection),
            lane_state.get("distance_to_crossroad_mm"),
        )
