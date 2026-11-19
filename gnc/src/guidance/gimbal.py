"""Line-of-sight gimbal geometry and rate limiting."""

from __future__ import annotations

import math

from .models import GimbalModel, GimbalState, Vec3


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def look_angles(position: Vec3, target: Vec3) -> tuple[float, float]:
    """Return world yaw and pitch from ``position`` to ``target``.

    Yaw is measured clockwise from north and pitch is positive upwards.
    """
    east = target[0] - position[0]
    north = target[1] - position[1]
    up = target[2] - position[2]
    return math.atan2(east, north), math.atan2(up, math.hypot(east, north))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def body_angles(
    world_yaw: float,
    world_pitch: float,
    aircraft_yaw: float,
    model: GimbalModel,
) -> tuple[float, float, float]:
    """Convert a world look target to body-frame yaw, pitch, and roll.

    The camera is commanded level in roll; the airframe is assumed level.
    """
    yaw = clamp(
        wrap_angle(world_yaw - aircraft_yaw), model.yaw_min_rad, model.yaw_max_rad
    )
    pitch = clamp(world_pitch, model.pitch_min_rad, model.pitch_max_rad)
    roll = clamp(0.0, model.roll_min_rad, model.roll_max_rad)
    return yaw, pitch, roll


def yaw_for_limits(
    world_yaw: float, aircraft_yaw: float, model: GimbalModel
) -> float:
    """Adjust aircraft yaw only when the target falls outside the gimbal range."""
    body_yaw = wrap_angle(world_yaw - aircraft_yaw)
    if body_yaw > model.yaw_max_rad:
        return wrap_angle(world_yaw - model.yaw_max_rad)
    if body_yaw < model.yaw_min_rad:
        return wrap_angle(world_yaw - model.yaw_min_rad)
    return aircraft_yaw


def is_settled(
    measured: GimbalState,
    desired_yaw: float,
    desired_pitch: float,
    desired_roll: float,
    model: GimbalModel,
    aircraft_yaw_rate_radps: float = 0.0,
) -> bool:
    """True when the measured camera is within the settle tolerances.

    ``aircraft_yaw_rate_radps`` is added to the measured gimbal yaw rate because
    the camera's world-frame yaw rate includes the airframe's yaw motion.
    """
    if abs(wrap_angle(desired_yaw - measured.yaw_rad)) > model.settle_tolerance_rad:
        return False
    world_yaw_rate = measured.yaw_rate_radps + aircraft_yaw_rate_radps
    if abs(world_yaw_rate) > model.settle_rate_tolerance_radps:
        return False
    if abs(desired_pitch - measured.pitch_rad) > model.settle_tolerance_rad:
        return False
    if abs(measured.pitch_rate_radps) > model.settle_rate_tolerance_radps:
        return False
    if abs(desired_roll - measured.roll_rad) > model.settle_tolerance_rad:
        return False
    if abs(measured.roll_rate_radps) > model.settle_rate_tolerance_radps:
        return False
    return True


def limit_yaw_change(previous: float, target: float, max_step: float) -> float:
    """Move yaw toward ``target`` by at most ``max_step`` radians."""
    delta = wrap_angle(target - previous)
    return wrap_angle(previous + max(-max_step, min(max_step, delta)))


def rate_command(
    error: float,
    measured_rate: float,
    max_rate: float,
    max_acceleration: float,
    dt: float,
    gain: float,
) -> float:
    """P-controlled rate command with an acceleration limit."""
    target = clamp(gain * error, -max_rate, max_rate)
    step = max_acceleration * dt
    return measured_rate + clamp(target - measured_rate, -step, step)
