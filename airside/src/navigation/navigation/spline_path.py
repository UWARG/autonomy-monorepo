"""
A smooth path through waypoints and pure-pursuit aiming along it.

The path is a centripetal Catmull-Rom spline.
Points are local ``(east, north, up)`` meters.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Sequence

Point = tuple[float, float, float]

# Spacing (meters) between polyline samples along the spline.
_SAMPLE_SPACING_M = 0.5
# Consecutive waypoints closer than this (meters) are treated as one.
_DUPLICATE_M = 0.1


def _lerp(a: Point, b: Point, weight: float) -> Point:
    return (
        a[0] + (b[0] - a[0]) * weight,
        a[1] + (b[1] - a[1]) * weight,
        a[2] + (b[2] - a[2]) * weight,
    )


def _segment(p0: Point, p1: Point, p2: Point, p3: Point, samples: int) -> list[Point]:
    """
    Samples the centripetal Catmull-Rom curve from ``p1`` to ``p2``
    (excluding ``p2``) with the Barry-Goldman pyramid.
    """

    t0 = 0.0
    t1 = t0 + math.sqrt(math.dist(p0, p1))
    t2 = t1 + math.sqrt(math.dist(p1, p2))
    t3 = t2 + math.sqrt(math.dist(p2, p3))
    points = []
    for i in range(samples):
        t = t1 + (t2 - t1) * i / samples
        a1 = _lerp(p0, p1, (t - t0) / (t1 - t0))
        a2 = _lerp(p1, p2, (t - t1) / (t2 - t1))
        a3 = _lerp(p2, p3, (t - t2) / (t3 - t2))
        b1 = _lerp(a1, a2, (t - t0) / (t2 - t0))
        b2 = _lerp(a2, a3, (t - t1) / (t3 - t1))
        points.append(_lerp(b1, b2, (t - t1) / (t2 - t1)))
    return points


class SplinePath:
    """
    A centripetal Catmull-Rom spline through ``waypoints``, sampled as a
    polyline. The ends are extended by reflecting the neighbouring waypoint,
    so the spline leaves the first and enters the last waypoint straight.
    """

    def __init__(self, waypoints: Sequence[Point]) -> None:
        if not waypoints:
            raise ValueError("a path needs at least one waypoint")
        self.waypoints = list(waypoints)
        # Indices into the deduplicated list, for each input waypoint.
        self._waypoint_of: list[int] = []
        controls: list[Point] = []
        for waypoint in waypoints:
            if not controls or math.dist(controls[-1], waypoint) > _DUPLICATE_M:
                controls.append(waypoint)
            self._waypoint_of.append(len(controls) - 1)

        # Polyline index of each deduplicated waypoint.
        self._knots: list[int] = []
        self.points: list[Point] = []
        if len(controls) > 1:
            first = _lerp(controls[1], controls[0], 2.0)
            last = _lerp(controls[-2], controls[-1], 2.0)
            padded = [first, *controls, last]
            for i in range(len(controls) - 1):
                self._knots.append(len(self.points))
                samples = max(1, math.ceil(math.dist(controls[i], controls[i + 1]) / _SAMPLE_SPACING_M))
                self.points.extend(_segment(*padded[i : i + 4], samples))
        self._knots.append(len(self.points))
        self.points.append(controls[-1])

        # Arc length (meters) at each polyline point.
        self.arc_m = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.arc_m.append(self.arc_m[-1] + math.dist(a, b))

    def arc_at_waypoint(self, index: int) -> float:
        """Arc length (meters) at input waypoint ``index``."""

        return self.arc_m[self._knots[self._waypoint_of[index]]]

    def point_at(self, arc_m: float) -> Point:
        """The point ``arc_m`` along the path, clamped to its ends."""

        if arc_m <= 0.0:
            return self.points[0]
        if arc_m >= self.arc_m[-1]:
            return self.points[-1]
        i = bisect.bisect_right(self.arc_m, arc_m) - 1
        span = self.arc_m[i + 1] - self.arc_m[i]
        return _lerp(self.points[i], self.points[i + 1], (arc_m - self.arc_m[i]) / span)

    def project(self, position: Point, from_arc_m: float, to_arc_m: float) -> float:
        """
        Arc length of the point on the path closest to ``position``,
        searching only between ``from_arc_m`` and ``to_arc_m``.
        """

        start = max(0, bisect.bisect_right(self.arc_m, from_arc_m) - 1)
        end = min(len(self.points) - 1, bisect.bisect_left(self.arc_m, to_arc_m))
        best_arc_m, best_distance_m = from_arc_m, math.inf
        for i in range(start, max(start, end)):
            a, b = self.points[i], self.points[i + 1]
            ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            length_sq = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2
            weight = 0.0
            if length_sq > 0.0:
                weight = sum((position[k] - a[k]) * ab[k] for k in range(3)) / length_sq
                weight = min(1.0, max(0.0, weight))
            distance_m = math.dist(position, _lerp(a, b, weight))
            if distance_m < best_distance_m:
                best_distance_m = distance_m
                best_arc_m = self.arc_m[i] + weight * (self.arc_m[i + 1] - self.arc_m[i])
        return min(max(best_arc_m, from_arc_m), to_arc_m)


def aim_point(
    path: SplinePath,
    target_index: int,
    position: Point,
    lookahead_m: float,
    corner_lookahead_m: float,
) -> Point:
    """
    Where to aim to follow ``path`` toward its waypoint ``target_index``, stopping at the end of the path.
    """

    def lookahead_for(to_waypoint_m: float) -> float:
        return min(lookahead_m, max(corner_lookahead_m, to_waypoint_m))

    waypoints = path.waypoints
    target_arc_m = path.arc_at_waypoint(target_index)
    if target_index == 0:
        to_target_m = math.dist(position, waypoints[0])
        ahead_m = lookahead_for(to_target_m)
        if to_target_m >= ahead_m:
            return waypoints[0]
        return path.point_at(target_arc_m + ahead_m - to_target_m)

    search_from_m = path.arc_at_waypoint(max(0, target_index - 2))
    search_to_m = path.arc_at_waypoint(min(len(waypoints) - 1, target_index + 1))
    along_m = path.project(position, search_from_m, search_to_m)
    next_waypoint_m = min(
        (
            path.arc_at_waypoint(index) - along_m
            for index in range(len(waypoints))
            if path.arc_at_waypoint(index) > along_m
        ),
        default=math.inf,
    )
    return path.point_at(along_m + lookahead_for(next_waypoint_m))
