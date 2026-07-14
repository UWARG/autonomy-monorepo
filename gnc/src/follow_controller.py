"""Person-follow control law: camera XYZ -> body-frame velocity setpoint"""

from dataclasses import dataclass, field
import math

from frames import camera_to_body


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class FollowConfig:
    """Tunable parameters for the follow controller (SI units, radians)."""

    standoff_m: float = 2.5
    hard_min_m: float = 1.5
    margin_m: float = 0.5
    v_max: float = 1.5
    a_brake: float = 1.0
    kp_range: float = 0.6
    kp_yaw: float = 1.2
    yaw_rate_max: float = 0.8
    kp_vertical: float = 0.5
    v_vertical_max: float = 0.5
    desired_vertical_offset_m: float = 0.0
    mount_pitch_rad: float = 0.0
    mount_roll_rad: float = 0.0


@dataclass(frozen=True)
class FollowSetpoint:
    """Body-FRD velocity command plus diagnostics (forward, right, down; rad/s)."""

    v_forward: float
    v_right: float
    v_down: float
    yaw_rate: float
    # diagnostics used by the sim, tests and logging
    range_3d_m: float = field(default=0.0)
    horiz_range_m: float = field(default=0.0)
    bearing_rad: float = field(default=0.0)
    range_error_m: float = field(default=0.0)
    v_brake_cap: float = field(default=0.0)


def braking_cap(range_error_m: float, a_brake: float, margin_m: float) -> float:
    """Largest approach speed from which the drone can still stop before the ring"""
    usable = max(range_error_m - margin_m, 0.0)
    return math.sqrt(2.0 * a_brake * usable)


def compute_setpoint(
    cam_x_mm: float,
    cam_y_mm: float,
    cam_z_mm: float,
    config: FollowConfig = FollowConfig(),
) -> FollowSetpoint:
    """Compute the body-frame velocity setpoint for one camera observation."""
    body = camera_to_body(
        cam_x_mm,
        cam_y_mm,
        cam_z_mm,
        mount_pitch_rad=config.mount_pitch_rad,
        mount_roll_rad=config.mount_roll_rad,
    )
    forward, right, down = body.x, body.y, body.z

    horiz_range = math.hypot(forward, right)
    range_3d = math.sqrt(forward * forward + right * right + down * down)
    bearing = math.atan2(right, forward)
    error = horiz_range - config.standoff_m

    v_brake_cap = braking_cap(error, config.a_brake, config.margin_m)
    approach_cap = min(config.v_max, v_brake_cap)

    # P-control on the standoff error.
    desired = config.kp_range * error
    v_forward = clamp(desired, -config.v_max, approach_cap)
    v_forward *= max(math.cos(bearing), 0.0)

    yaw_rate = clamp(config.kp_yaw * bearing, -config.yaw_rate_max, config.yaw_rate_max)

    vertical_offset = down - config.desired_vertical_offset_m
    v_down = clamp(
        config.kp_vertical * vertical_offset,
        -config.v_vertical_max,
        config.v_vertical_max,
    )

    return FollowSetpoint(
        v_forward=v_forward,
        v_right=0.0,  # v1: rely on yaw-centring; strafe disabled for smoothness
        v_down=v_down,
        yaw_rate=yaw_rate,
        range_3d_m=range_3d,
        horiz_range_m=horiz_range,
        bearing_rad=bearing,
        range_error_m=error,
        v_brake_cap=v_brake_cap,
    )
