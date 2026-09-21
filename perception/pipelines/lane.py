"""单帧视觉算法编排，隔离旧 LaneTracker 的可变状态。"""

import math

from ..contracts import FrameAnalysis, LaneMeasurement, LaneSampleStatus


class VisionPipeline:
    """将现有车道算法包装为不可变的单帧分析产品。"""

    def __init__(self, lane_tracker):
        self._lane_tracker = lane_tracker

    def analyze(self, frame, frame_id, captured_at_monotonic_ns):
        try:
            offset_mm, is_intersection, _ = self._lane_tracker.process(frame)
            lane_state = self._lane_tracker.last_lane_state or {}
        except Exception as error:
            return self._invalid(frame_id, captured_at_monotonic_ns, "pipeline exception: {}".format(error))

        if lane_state.get("frame_dropped", False):
            return self._invalid(
                frame_id,
                captured_at_monotonic_ns,
                lane_state.get("drop_reason") or "lane frame dropped",
                LaneSampleStatus.PROCESSING,
            )

        if offset_mm is None:
            return self._invalid(frame_id, captured_at_monotonic_ns, "lane offset unavailable")

        quality_score = float(lane_state.get("quality_score", 1.0))
        quality_score = max(0.0, min(1.0, quality_score))
        lane_measurement = LaneMeasurement(
            LaneSampleStatus.VALID,
            float(offset_mm),
            math.degrees(float(lane_state.get("lane_angle_rad", 0.0))),
            quality_score,
        )
        return FrameAnalysis(
            frame_id,
            captured_at_monotonic_ns,
            lane_measurement,
            bool(is_intersection),
            lane_state.get("distance_to_crossroad_mm"),
        )

    @staticmethod
    def _invalid(frame_id, captured_at_monotonic_ns, reason, status=LaneSampleStatus.NO_VALID_LANE):
        return FrameAnalysis(
            frame_id,
            captured_at_monotonic_ns,
            LaneMeasurement(status, None, None, 0.0, reason),
        )
