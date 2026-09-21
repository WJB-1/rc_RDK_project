# perception/algorithms/pidinet_lane.py
"""兼容层：所有实现在 lane/ 子包。

保留老版本的 re-export，外部 `from perception.algorithms.pidinet_lane import XXX`
不需要改。
"""

from .lane.constants import (
    ANGLE_TOL_DEG, NORMAL_DIST_TOL, MIN_LINE_LENGTH,
)
from .lane.line_detection import (
    thin_binary, postprocess_edge_probability, detect_lines,
    draw_lines, merge_lines, lines_to_mask, line_mask_coverage,
)
from .lane.line_geometry import (
    find_valid_lane_pairs, select_best_pair,
    infer_lane_pair_from_single_line, lane_offset_at_vehicle_cross_section,
    vehicle_in_lane_closure,
)
from .lane.ground_ipm_selector import GroundIPMLanePairSelector
from .lane.template_selector import TemplateDistanceLaneSelector

__all__ = [
    # 常量（向后兼容）
    "ANGLE_TOL_DEG", "NORMAL_DIST_TOL", "MIN_LINE_LENGTH",
    # 纯函数
    "thin_binary", "postprocess_edge_probability", "detect_lines",
    "draw_lines", "merge_lines", "lines_to_mask", "line_mask_coverage",
    "find_valid_lane_pairs", "select_best_pair",
    "infer_lane_pair_from_single_line", "lane_offset_at_vehicle_cross_section",
    "vehicle_in_lane_closure",
    # 选择器
    "GroundIPMLanePairSelector", "TemplateDistanceLaneSelector",
]