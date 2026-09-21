# perception/algorithms/lane/types.py
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class LanePipelineResult:
    """LanePipeline 的单帧输出。

    所有可视化 / debug 素材都放这里，LaneTracker 只做转发。
    """
    offset_mm: float
    is_intersection: bool
    debug_frame: np.ndarray

    lane_state: dict
    bev_mask: np.ndarray
    original_view: np.ndarray
    seg_mask: Optional[np.ndarray]

    timing: dict = field(default_factory=dict)
    debug_views: dict = field(default_factory=dict)
    debug_capture: dict = field(default_factory=dict)