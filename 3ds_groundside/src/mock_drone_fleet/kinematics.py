"""Motion model for mock drones."""

import math

from src.mock_drone_fleet.geometry import GeoPosition, Vector3, from_local, to_local


def step_toward(position: Vector3, target: Vector3, speed: float, dt: float) -> Vector3:
    """Move toward `target` at `speed` m/s for `dt` s, stopping exactly on it."""
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


def step_toward_geo(
    start: GeoPosition, target: GeoPosition, speed: float, dt: float
) -> GeoPosition:
    """Move from `start` toward `target` in a straight line at `speed` m/s for `dt` s.

    Works in metres east/north/up around `start`, and returns `target` exactly once
    it is reached.
    """
    local_target = to_local(start, target)
    offset = step_toward((0.0, 0.0, 0.0), local_target, speed, dt)
    if offset is local_target:  # step_toward hands back `target` itself on arrival
        return target
    return from_local(start, offset)
