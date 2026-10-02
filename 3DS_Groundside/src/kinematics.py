"""Motion model for mock drones."""

import math
from typing import Tuple

Vector3 = Tuple[float, float, float] #x, y, z


def step_toward(position: Vector3, target: Vector3, speed: float, dt: float) -> Vector3:
    """Move `position` in a straight line toward `target` at `speed` (m/s) for `dt` seconds.

    Returns `target` exactly if it is reachable within this step.
    """
    if speed < 0:
        raise ValueError(f"speed must be non-negative, got {speed}")
    if dt < 0:
        raise ValueError(f"dt must be non-negative, got {dt}")

    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dz = target[2] - position[2]
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    max_step = speed * dt
    if distance <= max_step:
        return target

    scale = max_step / distance
    return (
        position[0] + dx * scale,
        position[1] + dy * scale,
        position[2] + dz * scale,
    )
