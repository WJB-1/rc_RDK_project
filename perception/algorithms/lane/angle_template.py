"""Angle-conditioned six-line template reconstruction."""

from __future__ import annotations

from typing import Mapping


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
    """Return the six reference-line positions for a ground offset angle.

    The measured angle is folded to its absolute value and clamped to the
    0--30 degree calibration coverage. Each neighbouring gap uses its own
    fitted line, while all observed non-neighbour pairs constrain their sums.
    """
    gaps = neighbour_gaps_for_angle(angle_deg)

    x0 = gaps[("0", "1")] * 0.5
    x1 = -x0
    x2 = x0 + gaps[("2", "0")]
    x3 = x1 - gaps[("1", "3")]
    return {
        "0": x0,
        "1": x1,
        "2": x2,
        "3": x3,
        "4": x2 + gaps[("4", "2")],
        "5": x3 - gaps[("3", "5")],
    }
