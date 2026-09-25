"""Small, dependency-free 2D geometry helpers."""

from __future__ import annotations

import math

from .models import Point2D

TAU = 2.0 * math.pi


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)


def heading(a: Point2D, b: Point2D) -> float:
    return math.atan2(b.y - a.y, b.x - a.x)


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi)."""

    return (angle_rad + math.pi) % TAU - math.pi


def angular_distance(a_rad: float, b_rad: float) -> float:
    return abs(wrap_angle(a_rad - b_rad))


def project(start: Point2D, heading_rad: float, distance_m: float) -> Point2D:
    return Point2D(
        start.x + math.cos(heading_rad) * distance_m,
        start.y + math.sin(heading_rad) * distance_m,
    )


def point_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return distance(point, start)

    fraction = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    closest = Point2D(start.x + fraction * dx, start.y + fraction * dy)
    return distance(point, closest)
