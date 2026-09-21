# perception/algorithms/lane_tracker.py
from typing import Optional, Tuple

import numpy as np

from utils.logger import get_logger
from .lane.builders import build_lane_pipeline


class LaneTracker:
    """门面：初始化 pipeline，转发 process/reset，保存最近状态。"""

    def __init__(self, settings: dict):
        self.logger = get_logger()
        self.pipeline = build_lane_pipeline(settings)

        self.last_lane_state = None
        self.last_bev_mask = None
        self.last_original_view = None
        self.last_seg_mask = None
        self.last_timing = {}
        self.last_debug_views = {}
        self.last_debug_capture = {}

        self.debug = settings.get("debug", {}).get("show_video", True)

    def process(self, frame: Optional[np.ndarray]) -> Tuple[float, bool, np.ndarray]:
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            self.logger.warning("输入帧无效")
            return 0.0, False, np.zeros((240, 320, 3), dtype=np.uint8)

        result = self.pipeline.process(frame)

        self.last_lane_state = result.lane_state
        self.last_bev_mask = result.bev_mask
        self.last_original_view = result.original_view
        self.last_seg_mask = result.seg_mask
        self.last_timing = result.timing
        self.last_debug_views = result.debug_views
        self.last_debug_capture = result.debug_capture

        return result.offset_mm, result.is_intersection, result.debug_frame

    def reset(self):
        self.pipeline.reset()
        self.logger.info("LaneTracker 已重置")