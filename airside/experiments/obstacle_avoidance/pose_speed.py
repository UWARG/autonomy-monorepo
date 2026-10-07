"""Speed estimates from MAVROS pose measurements, not callback timing."""

from __future__ import annotations

import math


def horizontal_speed_mps(
    previous: tuple[float, float, float],
    current: tuple[float, float, float],
    *,
    minimum_interval_s: float = 0.08,
) -> float | None:
    """Return displacement / source-stamp interval, or no valid estimate."""

    if not all(math.isfinite(value) for value in (*previous, *current)):
        return None
    if previous[2] <= 0.0 or current[2] <= 0.0:
        return None
    interval_s = current[2] - previous[2]
    if interval_s < minimum_interval_s:
        return None
    return math.hypot(current[0] - previous[0], current[1] - previous[1]) / interval_s
