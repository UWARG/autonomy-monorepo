"""Data types for smooth viewpoint guidance and real-time tracking."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

Vec3 = tuple[float, float, float]


def _check_finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _check_vec(name: str, value: Vec3) -> None:
    if len(value) != 3:
        raise ValueError(f"{name} must have three components")
    for component in value:
        _check_finite(name, component)


class TrackerStatus(Enum):
    """Outcome of one tracker update."""

    TRACKING = "tracking"
    CAPTURE_READY = "capture_ready"
    CAPTURE_MISSED = "capture_missed"
    COMPLETE = "complete"


@dataclass(frozen=True)
class Viewpoint:
    """A path point and, optionally, a camera target."""

    position_m: Vec3
    look_at_m: Vec3 | None = None
    capture: bool = False
    capture_speed_mps: float | None = None

    def __post_init__(self) -> None:
        _check_vec("position_m", self.position_m)
        if self.look_at_m is not None:
            _check_vec("look_at_m", self.look_at_m)
        if self.capture and self.look_at_m is None:
            raise ValueError("capture viewpoints require look_at_m")
        if self.capture_speed_mps is not None and self.capture_speed_mps <= 0.0:
            raise ValueError("capture_speed_mps must be positive")


@dataclass(frozen=True)
class GuidanceLimits:
    """Velocity, acceleration, and capture constraints."""

    max_velocity_mps: float = 5.0
    max_acceleration_mps2: float = 1.5
    max_jerk_mps3: float | None = None
    capture_speed_mps: float = 0.5
    sample_period_s: float = 0.02
    capture_radius_m: float = 1.0
    capture_slowdown_m: float = 2.0
    max_deviation_m: float | None = None

    def __post_init__(self) -> None:
        if self.max_velocity_mps <= 0.0:
            raise ValueError("max_velocity_mps must be positive")
        if self.max_acceleration_mps2 <= 0.0:
            raise ValueError("max_acceleration_mps2 must be positive")
        if self.max_jerk_mps3 is not None and self.max_jerk_mps3 <= 0.0:
            raise ValueError("max_jerk_mps3 must be positive")
        if not 0.0 < self.capture_speed_mps <= self.max_velocity_mps:
            raise ValueError("capture_speed_mps must be in (0, max_velocity_mps]")
        if self.sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")
        if self.capture_radius_m <= 0.0:
            raise ValueError("capture_radius_m must be positive")
        if self.capture_slowdown_m < 0.0:
            raise ValueError("capture_slowdown_m must not be negative")
        if self.max_deviation_m is not None and self.max_deviation_m <= 0.0:
            raise ValueError("max_deviation_m must be positive")


@dataclass(frozen=True)
class GimbalModel:
    """Three-axis gimbal limits and settling tolerances.

    Yaw limits are relative to the airframe. Pitch and roll limits are relative
    to the horizon/level orientation.
    """

    yaw_min_rad: float = -math.pi
    yaw_max_rad: float = math.pi
    pitch_min_rad: float = -math.pi / 2.0
    pitch_max_rad: float = math.pi / 6.0
    roll_min_rad: float = -math.pi
    roll_max_rad: float = math.pi
    max_yaw_rate_radps: float = math.radians(60.0)
    max_pitch_rate_radps: float = math.radians(60.0)
    max_roll_rate_radps: float = math.radians(60.0)
    max_yaw_acceleration_radps2: float = math.radians(120.0)
    max_pitch_acceleration_radps2: float = math.radians(120.0)
    max_roll_acceleration_radps2: float = math.radians(120.0)
    settle_tolerance_rad: float = math.radians(2.0)
    settle_rate_tolerance_radps: float = math.radians(2.0)
    settle_time_s: float = 0.75

    def __post_init__(self) -> None:
        for name, low, high in (
            ("yaw", self.yaw_min_rad, self.yaw_max_rad),
            ("pitch", self.pitch_min_rad, self.pitch_max_rad),
            ("roll", self.roll_min_rad, self.roll_max_rad),
        ):
            _check_finite(f"{name} limit", low)
            _check_finite(f"{name} limit", high)
            if not low < high:
                raise ValueError(f"{name} limits must be increasing")

        for name, value in (
            ("max_yaw_rate_radps", self.max_yaw_rate_radps),
            ("max_pitch_rate_radps", self.max_pitch_rate_radps),
            ("max_roll_rate_radps", self.max_roll_rate_radps),
            ("max_yaw_acceleration_radps2", self.max_yaw_acceleration_radps2),
            ("max_pitch_acceleration_radps2", self.max_pitch_acceleration_radps2),
            ("max_roll_acceleration_radps2", self.max_roll_acceleration_radps2),
            ("settle_tolerance_rad", self.settle_tolerance_rad),
            ("settle_rate_tolerance_radps", self.settle_rate_tolerance_radps),
            ("settle_time_s", self.settle_time_s),
        ):
            _check_finite(name, value)
            if value < 0.0:
                raise ValueError(f"{name} must not be negative")
        for name, value in (
            ("max_yaw_rate_radps", self.max_yaw_rate_radps),
            ("max_pitch_rate_radps", self.max_pitch_rate_radps),
            ("max_roll_rate_radps", self.max_roll_rate_radps),
            ("max_yaw_acceleration_radps2", self.max_yaw_acceleration_radps2),
            ("max_pitch_acceleration_radps2", self.max_pitch_acceleration_radps2),
            ("max_roll_acceleration_radps2", self.max_roll_acceleration_radps2),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.settle_tolerance_rad <= 0.0:
            raise ValueError("settle_tolerance_rad must be positive")
        if self.settle_rate_tolerance_radps <= 0.0:
            raise ValueError("settle_rate_tolerance_radps must be positive")


@dataclass(frozen=True)
class GimbalState:
    """Measured body-frame gimbal angles and rates.

    Matches the body-frame attitude reported by ArduPilot's
    ``GIMBAL_DEVICE_ATTITUDE_STATUS``.
    """

    yaw_rad: float = 0.0
    pitch_rad: float = 0.0
    roll_rad: float = 0.0
    yaw_rate_radps: float = 0.0
    pitch_rate_radps: float = 0.0
    roll_rate_radps: float = 0.0


@dataclass(frozen=True)
class CapturePoint:
    """A capture viewpoint together with its closest reference sample."""

    index: int
    sample_index: int
    position_m: Vec3
    look_at_m: Vec3
    speed_mps: float


@dataclass(frozen=True)
class ReferenceTrajectory:
    """A time-sampled path that a tracker can follow."""

    time_s: list[float]
    position_m: list[Vec3]
    velocity_mps: list[Vec3]
    acceleration_mps2: list[Vec3]
    speed_mps: list[float]
    aim_targets_m: list[Vec3 | None]
    captures: list[CapturePoint]
    final_position_m: Vec3
    duration_s: float


@dataclass(frozen=True)
class TrackerConfig:
    """Feedback gains and tracking tolerances."""

    position_gain: float = 2.0
    velocity_gain: float = 4.0
    gimbal_gain_radps_per_rad: float = 4.0
    max_aircraft_yaw_rate_radps: float = math.radians(60.0)
    capture_speed_tolerance_mps: float = 0.1
    complete_radius_m: float = 1.0
    completion_margin_s: float = 1.0
    projection_window_samples: int = 30

    def __post_init__(self) -> None:
        if self.position_gain < 0.0 or self.velocity_gain < 0.0:
            raise ValueError("feedback gains must not be negative")
        if self.gimbal_gain_radps_per_rad < 0.0:
            raise ValueError("gimbal gain must not be negative")
        if self.max_aircraft_yaw_rate_radps <= 0.0:
            raise ValueError("max_aircraft_yaw_rate_radps must be positive")
        if self.capture_speed_tolerance_mps < 0.0:
            raise ValueError("capture speed tolerance must not be negative")
        if self.complete_radius_m <= 0.0:
            raise ValueError("complete_radius_m must be positive")
        if self.completion_margin_s < 0.0:
            raise ValueError("completion_margin_s must not be negative")
        if self.projection_window_samples < 1:
            raise ValueError("projection_window_samples must be positive")


@dataclass(frozen=True)
class TrackerCommand:
    """Raw guidance output for one control step.

    Aircraft yaw is earth-frame. Gimbal yaw, pitch, roll, and their rates are
    earth-frame commands; the caller converts them to ArduPilot gimbal-manager
    messages.
    """

    reference_position_m: Vec3
    velocity_mps: Vec3
    acceleration_mps2: Vec3
    yaw_rad: float
    gimbal_yaw_rad: float
    gimbal_pitch_rad: float
    gimbal_roll_rad: float
    gimbal_yaw_rate_radps: float
    gimbal_pitch_rate_radps: float
    gimbal_roll_rate_radps: float
    capture_ready: bool
    capture_index: int
    reference_time_s: float
    position_error_m: float
    status: TrackerStatus
