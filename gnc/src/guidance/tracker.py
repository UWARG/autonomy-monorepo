"""Real-time tracker for a reference viewpoint path."""

from __future__ import annotations

import math

from .gimbal import (
    body_angles,
    is_settled,
    limit_yaw_change,
    look_angles,
    rate_command,
    wrap_angle,
    yaw_for_limits,
)
from .models import (
    GimbalModel,
    GimbalState,
    GuidanceLimits,
    ReferenceTrajectory,
    TrackerCommand,
    TrackerConfig,
    TrackerStatus,
    Vec3,
)

_EPS = 1e-9
_YAW_SPEED_EPS = 0.1


class GuidanceTracker:
    """Follows a reference trajectory using measured position and gimbal state."""

    def __init__(
        self,
        reference: ReferenceTrajectory,
        limits: GuidanceLimits,
        gimbal: GimbalModel,
        config: TrackerConfig | None = None,
    ) -> None:
        if len(reference.time_s) < 2:
            raise ValueError("reference must contain at least two samples")
        if any(
            len(values) != len(reference.time_s)
            for values in (
                reference.position_m,
                reference.velocity_mps,
                reference.acceleration_mps2,
                reference.speed_mps,
                reference.aim_targets_m,
            )
        ):
            raise ValueError("reference arrays must have equal length")

        self._reference = reference
        self._limits = limits
        self._gimbal = gimbal
        self._config = config or TrackerConfig()
        self._reference_period = max(
            (reference.time_s[-1] - reference.time_s[0])
            / (len(reference.time_s) - 1),
            _EPS,
        )
        self._index = 0
        self._capture_index = 0
        self._settled_time_s = 0.0
        self._last_acceleration: Vec3 = (0.0, 0.0, 0.0)
        self._last_yaw = 0.0

    def reset(self, position_m: Vec3, velocity_mps: Vec3 = (0.0, 0.0, 0.0)) -> None:
        """Place the tracker at the reference sample nearest ``position_m``."""
        _check_vec("position_m", position_m)
        _check_vec("velocity_mps", velocity_mps)
        self._index = self._nearest_index(
            position_m, 0, len(self._reference.position_m) - 1
        )
        self._capture_index = 0
        self._settled_time_s = 0.0
        self._last_acceleration = (0.0, 0.0, 0.0)
        self._last_yaw = _heading(velocity_mps, 0.0)

    def mark_captured(self, index: int | None = None) -> bool:
        """Advance past the current capture after its photo is taken."""
        if self._capture_index >= len(self._reference.captures):
            return False
        current = self._reference.captures[self._capture_index]
        if index is not None and index != current.index:
            return False
        self._capture_index += 1
        self._settled_time_s = 0.0
        return True

    def step(
        self,
        position_m: Vec3,
        velocity_mps: Vec3,
        gimbal_state: GimbalState,
        dt_s: float,
        aircraft_yaw_rate_radps: float = 0.0,
    ) -> TrackerCommand:
        """Return the guidance command for one control step."""
        _check_vec("position_m", position_m)
        _check_vec("velocity_mps", velocity_mps)
        _check_gimbal_state(gimbal_state)
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("dt_s must be positive and finite")
        if not math.isfinite(aircraft_yaw_rate_radps):
            raise ValueError("aircraft_yaw_rate_radps must be finite")

        self._update_index(position_m, dt_s)
        index = self._index
        reference = self._reference
        position_error = _sub(reference.position_m[index], position_m)
        velocity_error = _sub(reference.velocity_mps[index], velocity_mps)

        if (
            _dist(position_m, reference.final_position_m)
            <= self._config.complete_radius_m
            and reference.time_s[index]
            >= reference.duration_s - self._config.completion_margin_s
        ):
            return self._command(
                index,
                position_m,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                gimbal_state,
                self._last_yaw,
                TrackerStatus.COMPLETE,
                dt_s=dt_s,
            )

        acceleration = _add(
            reference.acceleration_mps2[index],
            _add(
                _scale(position_error, self._config.position_gain),
                _scale(velocity_error, self._config.velocity_gain),
            ),
        )
        acceleration = _limit_norm(acceleration, self._limits.max_acceleration_mps2)
        velocity = _limit_norm(
            _add(velocity_mps, _scale(acceleration, dt_s)),
            self._limits.max_velocity_mps,
        )
        acceleration = _limit_norm(
            _scale(_sub(velocity, velocity_mps), 1.0 / dt_s),
            self._limits.max_acceleration_mps2,
        )
        if self._limits.max_jerk_mps3 is not None:
            acceleration = _limit_change(
                self._last_acceleration,
                acceleration,
                self._limits.max_jerk_mps3 * dt_s,
            )
            velocity = _add(velocity_mps, _scale(acceleration, dt_s))
        self._last_acceleration = acceleration

        target = reference.aim_targets_m[index]
        desired_aircraft_yaw = _heading(velocity, self._last_yaw)
        if target is not None:
            world_yaw, world_pitch = look_angles(position_m, target)
            desired_aircraft_yaw = yaw_for_limits(
                world_yaw, desired_aircraft_yaw, self._gimbal
            )

        aircraft_yaw = limit_yaw_change(
            self._last_yaw,
            desired_aircraft_yaw,
            self._config.max_aircraft_yaw_rate_radps * dt_s,
        )
        self._last_yaw = aircraft_yaw

        if target is not None:
            desired_yaw, desired_pitch, desired_roll = body_angles(
                world_yaw, world_pitch, aircraft_yaw, self._gimbal
            )
        else:
            desired_yaw = gimbal_state.yaw_rad
            desired_pitch = gimbal_state.pitch_rad
            desired_roll = gimbal_state.roll_rad

        status = TrackerStatus.TRACKING
        capture_ready = False
        capture_index = -1
        if self._capture_index < len(reference.captures):
            capture = reference.captures[self._capture_index]
            distance = _dist(position_m, capture.position_m)
            speed = _norm(velocity_mps)
            settled = is_settled(
                gimbal_state,
                desired_yaw,
                desired_pitch,
                desired_roll,
                self._gimbal,
                aircraft_yaw_rate_radps,
            )
            if (
                distance <= self._limits.capture_radius_m
                and speed
                <= capture.speed_mps + self._config.capture_speed_tolerance_mps
                and settled
            ):
                self._settled_time_s += dt_s
            else:
                self._settled_time_s = 0.0

            if self._settled_time_s >= self._gimbal.settle_time_s:
                capture_ready = True
                capture_index = capture.index
                status = TrackerStatus.CAPTURE_READY
            elif (
                index > capture.sample_index
                and distance > self._limits.capture_radius_m
            ):
                capture_index = capture.index
                status = TrackerStatus.CAPTURE_MISSED
                self._capture_index += 1
                self._settled_time_s = 0.0

        return self._command(
            index,
            position_m,
            velocity,
            acceleration,
            gimbal_state,
            aircraft_yaw,
            status,
            desired_yaw,
            desired_pitch,
            desired_roll,
            capture_ready,
            capture_index,
            dt_s=dt_s,
        )

    def _command(
        self,
        index: int,
        position_m: Vec3,
        velocity: Vec3,
        acceleration: Vec3,
        gimbal_state: GimbalState,
        aircraft_yaw: float,
        status: TrackerStatus,
        desired_yaw: float | None = None,
        desired_pitch: float | None = None,
        desired_roll: float | None = None,
        capture_ready: bool = False,
        capture_index: int = -1,
        dt_s: float | None = None,
    ) -> TrackerCommand:
        if desired_yaw is None:
            desired_yaw = gimbal_state.yaw_rad
        if desired_pitch is None:
            desired_pitch = gimbal_state.pitch_rad
        if desired_roll is None:
            desired_roll = gimbal_state.roll_rad

        rate_dt = dt_s if dt_s is not None else self._reference_period
        yaw_rate = rate_command(
            wrap_angle(desired_yaw - gimbal_state.yaw_rad),
            gimbal_state.yaw_rate_radps,
            self._gimbal.max_yaw_rate_radps,
            self._gimbal.max_yaw_acceleration_radps2,
            rate_dt,
            self._config.gimbal_gain_radps_per_rad,
        )
        pitch_rate = rate_command(
            desired_pitch - gimbal_state.pitch_rad,
            gimbal_state.pitch_rate_radps,
            self._gimbal.max_pitch_rate_radps,
            self._gimbal.max_pitch_acceleration_radps2,
            rate_dt,
            self._config.gimbal_gain_radps_per_rad,
        )
        roll_rate = rate_command(
            desired_roll - gimbal_state.roll_rad,
            gimbal_state.roll_rate_radps,
            self._gimbal.max_roll_rate_radps,
            self._gimbal.max_roll_acceleration_radps2,
            rate_dt,
            self._config.gimbal_gain_radps_per_rad,
        )

        return TrackerCommand(
            reference_position_m=self._reference.position_m[index],
            velocity_mps=velocity,
            acceleration_mps2=acceleration,
            yaw_rad=aircraft_yaw,
            gimbal_yaw_rad=desired_yaw,
            gimbal_pitch_rad=desired_pitch,
            gimbal_roll_rad=desired_roll,
            gimbal_yaw_rate_radps=yaw_rate,
            gimbal_pitch_rate_radps=pitch_rate,
            gimbal_roll_rate_radps=roll_rate,
            capture_ready=capture_ready,
            capture_index=capture_index,
            reference_time_s=self._reference.time_s[index],
            position_error_m=_dist(position_m, self._reference.position_m[index]),
            status=status,
        )

    def _update_index(self, position_m: Vec3, dt_s: float) -> None:
        samples = self._reference.position_m
        travel = self._limits.max_velocity_mps * dt_s
        extra = int(math.ceil(travel / max(self._reference_period, _EPS))) + 2
        window = max(self._config.projection_window_samples, extra)
        self._index = self._nearest_index(
            position_m,
            max(0, self._index - window),
            min(len(samples) - 1, self._index + window),
        )

    def _nearest_index(self, position_m: Vec3, low: int, high: int) -> int:
        points = self._reference.position_m
        return min(
            range(low, high + 1),
            key=lambda index: _distance_squared(position_m, points[index]),
        )


def _check_vec(name: str, value: Vec3) -> None:
    if len(value) != 3 or not all(math.isfinite(component) for component in value):
        raise ValueError(f"{name} must contain three finite numbers")


def _check_gimbal_state(state: GimbalState) -> None:
    for name, value in (
        ("yaw_rad", state.yaw_rad),
        ("pitch_rad", state.pitch_rad),
        ("roll_rad", state.roll_rad),
        ("yaw_rate_radps", state.yaw_rate_radps),
        ("pitch_rate_radps", state.pitch_rate_radps),
        ("roll_rate_radps", state.roll_rate_radps),
    ):
        if not math.isfinite(value):
            raise ValueError(f"gimbal_state.{name} must be finite")


def _heading(velocity: Vec3, previous: float) -> float:
    if _norm(velocity) < _YAW_SPEED_EPS:
        return previous
    return math.atan2(velocity[0], velocity[1])


def _limit_norm(vector: Vec3, limit: float) -> Vec3:
    length = _norm(vector)
    if length <= limit or length < _EPS:
        return vector
    return _scale(vector, limit / length)


def _limit_change(previous: Vec3, target: Vec3, limit: float) -> Vec3:
    delta = _sub(target, previous)
    length = _norm(delta)
    if length <= limit or length < _EPS:
        return target
    return _add(previous, _scale(delta, limit / length))


def _add(first: Vec3, second: Vec3) -> Vec3:
    return (first[0] + second[0], first[1] + second[1], first[2] + second[2])


def _sub(first: Vec3, second: Vec3) -> Vec3:
    return (first[0] - second[0], first[1] - second[1], first[2] - second[2])


def _scale(vector: Vec3, factor: float) -> Vec3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def _dot(first: Vec3, second: Vec3) -> float:
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2]


def _norm(vector: Vec3) -> float:
    return math.sqrt(_dot(vector, vector))


def _dist(first: Vec3, second: Vec3) -> float:
    return _norm(_sub(first, second))


def _distance_squared(first: Vec3, second: Vec3) -> float:
    delta = _sub(first, second)
    return _dot(delta, delta)
