# perception/algorithms/lane_tracker.py
"""LaneTracker 门面：构建 pipeline，转发 process/reset，缓存最近状态。"""
from typing import Optional, Tuple

import numpy as np

from utils.logger import get_logger
from .lane.builders import build_lane_pipeline
from .lane.undistort import build_undistort_parameters


class LaneTracker:
    """门面：真正的逻辑都在 LanePipeline。"""

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

    @property
    def edge_engine(self):
        return self.pipeline.edge_engine

    @property
    def semantic_engine(self):
        return self.pipeline.semantic_engine

    @property
    def semantic_gate_enabled(self) -> bool:
        return self.pipeline.semantic_gate_enabled

    @property
    def lane_selector(self):
        return self.pipeline.selector

    @property
    def ipm(self):
        return self.pipeline.ipm

    @property
    def physical_track_width_mm(self) -> float:
        return self.pipeline.cfg.lane_width_mm

    def set_semantic_gate(self, enabled: bool) -> bool:
        self.pipeline.semantic_gate_enabled = bool(enabled) and self.semantic_engine is not None
        self.lane_selector.semantic_gate = self.pipeline.semantic_gate_enabled
        return self.pipeline.semantic_gate_enabled

    def set_debug_capture_enabled(self, enabled: bool) -> bool:
        return self.pipeline.set_debug_capture_enabled(enabled)

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
