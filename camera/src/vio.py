"""Pose estimates from visual-inertial odometry."""

from dataclasses import dataclass


@dataclass(frozen=True)
class VioPose:
    """Position in metres and orientation in the SDK's local frame."""

    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    timestamp_s: float | None
    received_monotonic_s: float