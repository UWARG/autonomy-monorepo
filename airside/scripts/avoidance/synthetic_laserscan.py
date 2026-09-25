"""Dependency-light geometry for the airside synthetic LaserScan source."""

from __future__ import annotations

import math
from collections.abc import Sequence


def wall_scan_ranges(
    *,
    east_m: float,
    north_m: float,
    yaw_enu_rad: float,
    wall_north_m: float,
    wall_half_width_m: float,
    angle_min_rad: float,
    angle_increment_rad: float,
    beam_count: int,
    range_max_m: float,
) -> tuple[float, ...]:
    """Ray-cast a finite east-west wall into a ROS FLU LaserScan."""

    if beam_count <= 0:
        raise ValueError("beam_count must be positive")
    if angle_increment_rad <= 0.0 or range_max_m <= 0.0:
        raise ValueError("scan increment and maximum range must be positive")

    ranges: list[float] = []
    for index in range(beam_count):
        body_angle_rad = angle_min_rad + index * angle_increment_rad
        world_angle_rad = yaw_enu_rad + body_angle_rad
        direction_east = math.cos(world_angle_rad)
        direction_north = math.sin(world_angle_rad)
        if abs(direction_north) <= 1e-9:
            ranges.append(math.inf)
            continue
        distance_m = (wall_north_m - north_m) / direction_north
        hit_east_m = east_m + distance_m * direction_east
        if (
            distance_m <= 0.0
            or distance_m > range_max_m
            or abs(hit_east_m) > wall_half_width_m
        ):
            ranges.append(math.inf)
        else:
            ranges.append(distance_m)
    return tuple(ranges)


def point_to_wall_distance_m(
    east_m: float,
    north_m: float,
    *,
    wall_north_m: float,
    wall_half_width_m: float,
) -> float:
    """Return Euclidean distance to a finite east-west wall segment."""

    east_error_m = max(abs(east_m) - wall_half_width_m, 0.0)
    north_error_m = wall_north_m - north_m
    return math.hypot(east_error_m, north_error_m)


def crossed_wall_segment(
    previous: Sequence[float],
    current: Sequence[float],
    *,
    wall_north_m: float,
    wall_half_width_m: float,
) -> bool:
    """Return whether an ENU motion segment crossed through the wall."""

    previous_east, previous_north = previous
    current_east, current_north = current
    if not (previous_north < wall_north_m <= current_north):
        return False
    north_delta = current_north - previous_north
    if north_delta <= 0.0:
        return False
    fraction = (wall_north_m - previous_north) / north_delta
    crossing_east = previous_east + fraction * (current_east - previous_east)
    return abs(crossing_east) <= wall_half_width_m
