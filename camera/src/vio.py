"""Pose estimates from visual-inertial odometry."""

from dataclasses import dataclass


@dataclass(frozen=True)
class VioPose:
    """Metres and an xyzw quaternion in the SDK's local frame.

    timestamp_s is the SDK Buffer timestamp, or None if unset. Host receipt
    time is not sensor latency or tracking confidence. No body/NED/GPS frame
    conversion is performed. Poses are independent of RGB/depth frames.
    """

    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    timestamp_s: float | None
    received_monotonic_s: float
