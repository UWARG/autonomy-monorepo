"""Smooth reference path and speed profile through ordered viewpoints."""

from __future__ import annotations

import bisect
import math

from .models import (
    CapturePoint,
    GuidanceLimits,
    ReferenceTrajectory,
    Vec3,
    Viewpoint,
)

_DS_M = 0.05
_EPS = 1e-9


def build_reference(
    viewpoints: list[Viewpoint], limits: GuidanceLimits
) -> ReferenceTrajectory:
    """Build a C2 reference path and time profile through ``viewpoints``.

    The path is a clamped cubic spline parameterised by chord length. Speeds are
    limited by ``limits.max_velocity_mps``, ``limits.max_acceleration_mps2`` and
    the curvature of the spline. Capture viewpoints receive a lower speed cap
    over ``limits.capture_slowdown_m``.
    """

    _validate_viewpoints(viewpoints, limits)

    points = [viewpoint.position_m for viewpoint in viewpoints]
    path = _ClampedCubic(points)
    path_points, tangents, curvature_vectors, curvature = _resample_path(path)

    deviation = _max_deviation(path_points, points)
    if limits.max_deviation_m is not None and deviation > limits.max_deviation_m:
        raise ValueError(
            f"reference deviates {deviation:.3f} m from the waypoint polyline; "
            f"limit is {limits.max_deviation_m:.3f} m"
        )

    capture_viewpoints = [viewpoint for viewpoint in viewpoints if viewpoint.capture]
    capture_path_indices = _ordered_indices(
        path_points, [viewpoint.position_m for viewpoint in capture_viewpoints]
    )
    capture_speeds = [
        viewpoint.capture_speed_mps or limits.capture_speed_mps
        for viewpoint in capture_viewpoints
    ]
    speeds = _speed_profile(
        curvature,
        capture_path_indices,
        capture_speeds,
        limits,
    )
    accelerations = _path_accelerations(
        tangents, curvature_vectors, speeds, _DS_M
    )

    time_s, position_m, velocity_mps, acceleration_mps2 = _sample_in_time(
        path_points,
        tangents,
        accelerations,
        speeds,
        limits.sample_period_s,
    )

    aim_targets = _aim_targets(viewpoints, position_m)
    captures = _capture_points(viewpoints, position_m, limits)
    return ReferenceTrajectory(
        time_s=time_s,
        position_m=position_m,
        velocity_mps=velocity_mps,
        acceleration_mps2=acceleration_mps2,
        speed_mps=[_norm(velocity) for velocity in velocity_mps],
        aim_targets_m=aim_targets,
        captures=captures,
        final_position_m=position_m[-1],
        duration_s=time_s[-1],
    )


def _validate_viewpoints(
    viewpoints: list[Viewpoint], limits: GuidanceLimits
) -> None:
    if len(viewpoints) < 2:
        raise ValueError("at least two viewpoints are required")
    for first, second in zip(viewpoints, viewpoints[1:]):
        if _dist(first.position_m, second.position_m) < 1e-6:
            raise ValueError("consecutive viewpoints must be distinct")
    for viewpoint in viewpoints:
        if (
            viewpoint.capture_speed_mps is not None
            and viewpoint.capture_speed_mps > limits.max_velocity_mps
        ):
            raise ValueError(
                "capture_speed_mps must not exceed max_velocity_mps"
            )


class _ClampedCubic:
    def __init__(self, points: list[Vec3]) -> None:
        self.points = points
        count = len(points)
        self.parameter = [0.0]
        for start, end in zip(points, points[1:]):
            self.parameter.append(self.parameter[-1] + max(_dist(start, end), _EPS))

        self.second = [[0.0] * count for _ in range(3)]
        if count == 2:
            return

        spacing = [
            self.parameter[index + 1] - self.parameter[index]
            for index in range(count - 1)
        ]
        for axis in range(3):
            lower = [0.0] * count
            diagonal = [1.0] * count
            upper = [0.0] * count
            right = [0.0] * count

            for index in range(1, count - 1):
                lower[index] = spacing[index - 1]
                diagonal[index] = 2.0 * (spacing[index - 1] + spacing[index])
                upper[index] = spacing[index]
                right[index] = 6.0 * (
                    (points[index + 1][axis] - points[index][axis]) / spacing[index]
                    - (points[index][axis] - points[index - 1][axis])
                    / spacing[index - 1]
                )

            diagonal[0] = 2.0
            upper[0] = 1.0
            right[0] = 0.0
            lower[-1] = 1.0
            diagonal[-1] = 2.0
            right[-1] = 0.0

            self.second[axis] = _solve_tridiagonal(lower, diagonal, upper, right)

    def evaluate(self, segment: int, fraction: float) -> Vec3:
        start = self.parameter[segment]
        end = self.parameter[segment + 1]
        spacing = end - start
        t = start + fraction * spacing
        left = (end - t) / spacing
        right = (t - start) / spacing

        result = []
        for axis in range(3):
            value = (
                left * self.points[segment][axis]
                + right * self.points[segment + 1][axis]
                + (
                    (left**3 - left) * self.second[axis][segment]
                    + (right**3 - right) * self.second[axis][segment + 1]
                )
                * spacing**2
                / 6.0
            )
            result.append(value)
        return (result[0], result[1], result[2])


def _solve_tridiagonal(
    lower: list[float],
    diagonal: list[float],
    upper: list[float],
    right: list[float],
) -> list[float]:
    count = len(diagonal)
    for index in range(1, count):
        factor = lower[index] / diagonal[index - 1]
        diagonal[index] -= factor * upper[index - 1]
        right[index] -= factor * right[index - 1]

    result = [0.0] * count
    result[-1] = right[-1] / diagonal[-1]
    for index in range(count - 2, -1, -1):
        result[index] = (
            right[index] - upper[index] * result[index + 1]
        ) / diagonal[index]
    return result


def _resample_path(
    path: _ClampedCubic,
) -> tuple[list[Vec3], list[Vec3], list[Vec3], list[float]]:
    raw_points: list[Vec3] = []
    raw_distance: list[float] = []
    for segment in range(len(path.points) - 1):
        length = _dist(path.points[segment], path.points[segment + 1])
        steps = max(2, int(math.ceil(length / 0.02)))
        for step in range(steps + 1):
            point = path.evaluate(segment, step / steps)
            if raw_points:
                raw_distance.append(raw_distance[-1] + _dist(raw_points[-1], point))
            else:
                raw_distance.append(0.0)
            raw_points.append(point)

    positions: list[Vec3] = []
    distances: list[float] = []
    target = 0.0
    index = 1
    while target <= raw_distance[-1] + _EPS:
        while index < len(raw_distance) and raw_distance[index] < target:
            index += 1
        if index >= len(raw_distance):
            break
        low = raw_distance[index - 1]
        high = raw_distance[index]
        fraction = 0.0 if high == low else (target - low) / (high - low)
        positions.append(
            _interpolate(raw_points[index - 1], raw_points[index], fraction)
        )
        distances.append(target)
        target += _DS_M

    if _dist(positions[-1], raw_points[-1]) > _EPS:
        positions.append(raw_points[-1])
        distances.append(raw_distance[-1])

    tangents = []
    curvature_vectors = []
    for index in range(len(positions)):
        before = positions[max(0, index - 1)]
        after = positions[min(len(positions) - 1, index + 1)]
        tangents.append(_unit(_sub(after, before)))

    for index in range(len(positions)):
        if index == 0 or index == len(positions) - 1:
            curvature_vectors.append((0.0, 0.0, 0.0))
            continue
        span = distances[index + 1] - distances[index - 1]
        curvature_vectors.append(
            _scale(_sub(tangents[index + 1], tangents[index - 1]), 1.0 / span)
        )

    return positions, tangents, curvature_vectors, [
        _norm(vector) for vector in curvature_vectors
    ]


def _speed_profile(
    curvature: list[float],
    capture_indices: list[int],
    capture_speeds: list[float],
    limits: GuidanceLimits,
) -> list[float]:
    speed_limit = [
        min(
            limits.max_velocity_mps,
            math.sqrt(limits.max_acceleration_mps2 / max(kappa, _EPS)),
        )
        for kappa in curvature
    ]

    slowdown = int(math.ceil(limits.capture_slowdown_m / _DS_M))
    for capture_index, capture_speed in zip(capture_indices, capture_speeds):
        low = max(0, capture_index - slowdown)
        high = min(len(speed_limit), capture_index + slowdown + 1)
        for index in range(low, high):
            speed_limit[index] = min(speed_limit[index], capture_speed)

    # Keep a small margin for discrete integration between path samples.
    acceleration = 0.98 * limits.max_acceleration_mps2
    speeds = list(speed_limit)
    speeds[0] = 0.0
    speeds[-1] = 0.0

    for index in range(len(speeds) - 1):
        lateral = curvature[index] * speeds[index] ** 2
        tangential = math.sqrt(max(acceleration**2 - lateral**2, 0.0))
        speeds[index + 1] = min(
            speed_limit[index + 1],
            math.sqrt(speeds[index] ** 2 + 2.0 * tangential * _DS_M),
        )

    speeds[-1] = 0.0
    for index in range(len(speeds) - 2, -1, -1):
        lateral = curvature[index + 1] * speeds[index + 1] ** 2
        tangential = math.sqrt(max(acceleration**2 - lateral**2, 0.0))
        speeds[index] = min(
            speeds[index],
            math.sqrt(speeds[index + 1] ** 2 + 2.0 * tangential * _DS_M),
        )

    return speeds


def _path_accelerations(
    tangents: list[Vec3],
    curvature_vectors: list[Vec3],
    speeds: list[float],
    spacing: float,
) -> list[Vec3]:
    result = []
    for index in range(len(tangents)):
        if index == 0:
            tangential = (speeds[1] ** 2 - speeds[0] ** 2) / (2.0 * spacing)
        elif index == len(tangents) - 1:
            tangential = (
                speeds[-1] ** 2 - speeds[-2] ** 2
            ) / (2.0 * spacing)
        else:
            tangential = (
                speeds[index + 1] ** 2 - speeds[index - 1] ** 2
            ) / (4.0 * spacing)
        normal = _scale(curvature_vectors[index], speeds[index] ** 2)
        result.append(_add(_scale(tangents[index], tangential), normal))
    return result


def _sample_in_time(
    positions: list[Vec3],
    tangents: list[Vec3],
    accelerations: list[Vec3],
    speeds: list[float],
    period: float,
) -> tuple[list[float], list[Vec3], list[Vec3], list[Vec3]]:
    path_time = [0.0]
    for index in range(len(speeds) - 1):
        duration = 2.0 * _DS_M / max(speeds[index] + speeds[index + 1], 1e-6)
        path_time.append(path_time[-1] + duration)

    duration = path_time[-1]
    count = max(1, int(math.ceil(duration / period)))
    sample_times = [min(index * period, duration) for index in range(count + 1)]
    if sample_times[-1] < duration:
        sample_times.append(duration)

    output_positions = []
    output_velocities = []
    output_accelerations = []
    for time in sample_times:
        index = min(bisect.bisect_right(path_time, time) - 1, len(path_time) - 2)
        low = path_time[index]
        high = path_time[index + 1]
        fraction = 0.0 if high == low else (time - low) / (high - low)
        output_positions.append(
            _interpolate(positions[index], positions[index + 1], fraction)
        )
        velocity = _scale(tangents[index], speeds[index])
        next_velocity = _scale(tangents[index + 1], speeds[index + 1])
        output_velocities.append(_interpolate(velocity, next_velocity, fraction))
        output_accelerations.append(
            _interpolate(accelerations[index], accelerations[index + 1], fraction)
        )

    return sample_times, output_positions, output_velocities, output_accelerations


def _aim_targets(
    viewpoints: list[Viewpoint], positions: list[Vec3]
) -> list[Vec3 | None]:
    targets = [
        (index, viewpoint.look_at_m)
        for index, viewpoint in enumerate(viewpoints)
        if viewpoint.look_at_m is not None
    ]
    if not targets:
        return [None] * len(positions)

    target_indices = _ordered_indices(
        positions,
        [
            viewpoint.position_m
            for viewpoint in viewpoints
            if viewpoint.look_at_m is not None
        ],
    )
    result: list[Vec3 | None] = []
    target = 0
    for index in range(len(positions)):
        while target + 1 < len(targets) and target_indices[target] < index:
            target += 1
        result.append(targets[target][1])
    return result


def _capture_points(
    viewpoints: list[Viewpoint], positions: list[Vec3], limits: GuidanceLimits
) -> list[CapturePoint]:
    entries = [
        (index, viewpoint)
        for index, viewpoint in enumerate(viewpoints)
        if viewpoint.capture
    ]
    sample_indices = _ordered_indices(
        positions, [viewpoint.position_m for _, viewpoint in entries]
    )
    return [
        CapturePoint(
            index=index,
            sample_index=sample_index,
            position_m=viewpoint.position_m,
            look_at_m=viewpoint.look_at_m,
            speed_mps=viewpoint.capture_speed_mps or limits.capture_speed_mps,
        )
        for (index, viewpoint), sample_index in zip(entries, sample_indices)
    ]


def _max_deviation(points: list[Vec3], polyline: list[Vec3]) -> float:
    return max(
        min(
            _point_segment_distance(point, start, end)
            for start, end in zip(polyline, polyline[1:])
        )
        for point in points
    )


def _ordered_indices(points: list[Vec3], targets: list[Vec3]) -> list[int]:
    """Map ordered targets to path samples without jumping to an earlier pass."""
    indices = []
    cursor = 0
    for target in targets:
        if cursor >= len(points):
            cursor = len(points) - 1
        index = min(
            range(cursor, len(points)),
            key=lambda candidate: _distance_squared(points[candidate], target),
        )
        indices.append(index)
        cursor = index + 1
    return indices


def _point_segment_distance(point: Vec3, start: Vec3, end: Vec3) -> float:
    segment = _sub(end, start)
    length_squared = _dot(segment, segment)
    if length_squared < _EPS:
        return _dist(point, start)
    fraction = max(0.0, min(1.0, _dot(_sub(point, start), segment) / length_squared))
    return _dist(point, _add(start, _scale(segment, fraction)))


def _interpolate(start: Vec3, end: Vec3, fraction: float) -> Vec3:
    return (
        start[0] + fraction * (end[0] - start[0]),
        start[1] + fraction * (end[1] - start[1]),
        start[2] + fraction * (end[2] - start[2]),
    )


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


def _unit(vector: Vec3) -> Vec3:
    length = _norm(vector)
    return (0.0, 0.0, 0.0) if length < _EPS else _scale(vector, 1.0 / length)


def _dist(first: Vec3, second: Vec3) -> float:
    return _norm(_sub(first, second))


def _distance_squared(first: Vec3, second: Vec3) -> float:
    delta = _sub(first, second)
    return _dot(delta, delta)
