"""Angle-conditioned six-line template reconstruction."""

from __future__ import annotations

from typing import Mapping

from .constants import TEMPLATE_LINE_ORDER, TEMPLATE_LINE_X_MM


MAX_SUPPORTED_ANGLE_DEG = 30.0
NEIGHBOUR_GAP_LINEAR_MODELS = {
    ("4", "2"): (29.00663, -0.131739),
    ("2", "0"): (39.03150, 0.382111),
    ("0", "1"): (208.19995, -0.631291),
    ("1", "3"): (55.27814, -0.939901),
    ("3", "5"): (27.63995, -0.081408),
}


def neighbour_gaps_for_angle(angle_deg: float) -> Mapping[tuple[str, str], float]:
    """Return the five fitted adjacent-line distances for a ground angle."""
    angle = min(abs(float(angle_deg)), MAX_SUPPORTED_ANGLE_DEG)
    return {
        pair: intercept_mm + slope_mm_per_deg * angle
        for pair, (intercept_mm, slope_mm_per_deg)
        in NEIGHBOUR_GAP_LINEAR_MODELS.items()
    }


def template_positions_for_angle(angle_deg: float) -> Mapping[str, float]:
    """Return a left-origin template with angle-corrected adjacent gaps."""
    angle_gaps = neighbour_gaps_for_angle(angle_deg)
    zero_gaps = neighbour_gaps_for_angle(0.0)
    positions = {TEMPLATE_LINE_ORDER[0]: 0.0}
    for left_id, right_id in zip(TEMPLATE_LINE_ORDER, TEMPLATE_LINE_ORDER[1:]):
        model_pair = (right_id, left_id)
        baseline_gap = TEMPLATE_LINE_X_MM[right_id] - TEMPLATE_LINE_X_MM[left_id]
        corrected_gap = baseline_gap + angle_gaps[model_pair] - zero_gaps[model_pair]
        positions[right_id] = positions[left_id] + corrected_gap
    return positions
