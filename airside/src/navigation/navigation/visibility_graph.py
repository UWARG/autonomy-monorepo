"""
Obstacle avoidance by shortest path over a visibility graph.

Works in a flat local frame of ``(east, north)`` meters. Avoidance is
horizontal only.

Each obstacle is surrounded by a keep-away zone shaped like a capsule: a circle
around the obstacle, stretched forwards along its direction of travel. Corner
points are placed around every zone; the planner joins up the drone, the
destination and the corners that can see each other without crossing a zone,
and searches that graph with A* for the shortest way through.
"""

from __future__ import annotations

import dataclasses
import enum
import heapq
import math

from navigation.constants import (
    DETOUR_SETPOINT_LEAD_M,
    ESCAPE_DISTANCE_M,
    FORWARD_ZONE_LOOKAHEAD_S,
    FORWARD_ZONE_MAX_EXTENSION_M,
    FORWARD_ZONE_MIN_SPEED_MPS,
    KEEP_AWAY_MARGIN_M,
    SIDE_SWITCH_PENALTY_M,
    ZONE_CORNER_COUNT,
    ZONE_CORNER_STANDOFF_M,
)


Point = tuple[float, float]

# Distances below this (meters) count as zero.
_EPSILON_M = 1e-6


def _bearing_unit_vector(bearing_deg: float) -> Point:
    """
    ``(east, north)`` unit vector of a compass bearing (0 at north, clockwise).
    """

    bearing_rad = math.radians(bearing_deg)
    return math.sin(bearing_rad), math.cos(bearing_rad)


def _bearing_deg(start: Point, end: Point) -> float:
    return math.degrees(math.atan2(end[0] - start[0], end[1] - start[1]))


def _project(start: Point, bearing_deg: float, distance_m: float) -> Point:
    east, north = _bearing_unit_vector(bearing_deg)
    return start[0] + east * distance_m, start[1] + north * distance_m


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _closest_point_on_segment(point: Point, start: Point, end: Point) -> Point:
    delta_east = end[0] - start[0]
    delta_north = end[1] - start[1]
    length_squared = delta_east**2 + delta_north**2
    if length_squared < _EPSILON_M**2:
        return start

    fraction = (
        (point[0] - start[0]) * delta_east + (point[1] - start[1]) * delta_north
    ) / length_squared
    fraction = min(1.0, max(0.0, fraction))
    return start[0] + fraction * delta_east, start[1] + fraction * delta_north


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    return _distance(point, _closest_point_on_segment(point, start, end))


def _cross(origin: Point, a: Point, b: Point) -> float:
    return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (
        b[0] - origin[0]
    )


def _segments_cross(a_start: Point, a_end: Point, b_start: Point, b_end: Point) -> bool:
    """
    True if the segments properly cross. Touching and collinear overlaps are
    left to the endpoint distances in ``_segment_segment_distance``.
    """

    return (
        _cross(a_start, a_end, b_start) * _cross(a_start, a_end, b_end) < 0.0
        and _cross(b_start, b_end, a_start) * _cross(b_start, b_end, a_end) < 0.0
    )


def _segment_segment_distance(
    a_start: Point, a_end: Point, b_start: Point, b_end: Point
) -> float:
    if _segments_cross(a_start, a_end, b_start, b_end):
        return 0.0

    return min(
        _point_segment_distance(a_start, b_start, b_end),
        _point_segment_distance(a_end, b_start, b_end),
        _point_segment_distance(b_start, a_start, a_end),
        _point_segment_distance(b_end, a_start, a_end),
    )


@dataclasses.dataclass(frozen=True)
class KeepAwayZone:
    """
    Capsule-shaped keep-away zone: everything within ``radius_m`` of the
    segment from ``tail`` (the obstacle itself) to ``nose`` (ahead of it).
    """

    tail: Point
    nose: Point
    radius_m: float

    @classmethod
    def from_obstacle(
        cls,
        position: Point,
        horizontal_keep_away_m: float,
        speed_mps: float,
        direction_deg: float,
    ) -> KeepAwayZone:
        """
        Zone around an obstacle at ``position`` moving at ``speed_mps`` along
        the compass bearing ``direction_deg``.
        """

        radius_m = max(0.0, horizontal_keep_away_m) + KEEP_AWAY_MARGIN_M

        extension_m = 0.0
        if speed_mps >= FORWARD_ZONE_MIN_SPEED_MPS:
            extension_m = min(
                speed_mps * FORWARD_ZONE_LOOKAHEAD_S, FORWARD_ZONE_MAX_EXTENSION_M
            )

        return cls(
            tail=position,
            nose=_project(position, direction_deg, extension_m),
            radius_m=radius_m,
        )

    def point_clearance_m(self, point: Point) -> float:
        """
        Distance from ``point`` to the edge of the zone; negative inside it.
        """

        return _point_segment_distance(point, self.tail, self.nose) - self.radius_m

    def segment_clearance_m(self, start: Point, end: Point) -> float:
        """
        Smallest distance from the path ``start`` -> ``end`` to the edge of the
        zone; negative if the path enters it.
        """

        return (
            _segment_segment_distance(start, end, self.tail, self.nose) - self.radius_m
        )

    def blocks(self, start: Point, end: Point) -> bool:
        """
        True if the path ``start`` -> ``end`` enters the zone.
        """

        # Cheap reject first: most paths are nowhere near the zone
        center = (
            (self.tail[0] + self.nose[0]) / 2.0,
            (self.tail[1] + self.nose[1]) / 2.0,
        )
        reach_m = _distance(self.tail, self.nose) / 2.0 + self.radius_m
        if _point_segment_distance(center, start, end) >= reach_m:
            return False

        return self.segment_clearance_m(start, end) < 0.0

    def escape_bearing_deg(self, point: Point) -> float:
        """
        Compass bearing that leaves the zone fastest from ``point``.
        """

        closest = _closest_point_on_segment(point, self.tail, self.nose)
        if _distance(closest, point) < _EPSILON_M:
            # Sitting on the obstacle's path: step off it sideways
            return _bearing_deg(self.tail, self.nose) + 90.0

        return _bearing_deg(closest, point)

    def pushed_out(self, point: Point) -> Point:
        """
        The nearest point to ``point`` that is clear of the zone by the corner
        standoff.
        """

        closest = _closest_point_on_segment(point, self.tail, self.nose)
        return _project(
            closest,
            self.escape_bearing_deg(point),
            self.radius_m + ZONE_CORNER_STANDOFF_M,
        )

    def corners(self) -> list[Point]:
        """
        Corners of a polygon drawn around the zone, ``ZONE_CORNER_STANDOFF_M``
        outside it at its closest.
        """

        heading_deg = _bearing_deg(self.tail, self.nose)
        step_deg = 360.0 / ZONE_CORNER_COUNT
        # Corners sit further out than the sides of the polygon they make
        corner_radius_m = (self.radius_m + ZONE_CORNER_STANDOFF_M) / math.cos(
            math.radians(step_deg / 2.0)
        )

        corners = []
        for index in range(ZONE_CORNER_COUNT):
            offset_deg = -180.0 + (index + 0.5) * step_deg
            # Front half goes round the nose, back half round the tail
            center = self.nose if abs(offset_deg) < 90.0 else self.tail
            corners.append(_project(center, heading_deg + offset_deg, corner_radius_m))
        return corners


class PlanStatus(enum.Enum):
    """
    What the planner decided to do.
    """

    DIRECT = "direct"  # Way to the destination is clear
    DETOUR = "detour"  # Going around a zone
    ESCAPE = "escape"  # Inside a zone, getting out of it
    HOLD = "hold"  # No way through, staying put


@dataclasses.dataclass(frozen=True)
class Plan:
    """
    Where to fly next. ``point`` is the destination itself when ``status`` is
    ``DIRECT``.
    """

    point: Point
    status: PlanStatus


def _is_clear(start: Point, end: Point, zones: list[KeepAwayZone]) -> bool:
    return not any(zone.blocks(start, end) for zone in zones)


def _side(start: Point, goal: Point, point: Point) -> int:
    """
    Which side of the line ``start`` -> ``goal`` a point is on: +1 left, -1
    right, 0 on it.
    """

    cross = _cross(start, goal, point)
    if abs(cross) < _EPSILON_M:
        return 0
    return 1 if cross > 0.0 else -1


def _shortest_path(
    start: Point, goal: Point, zones: list[KeepAwayZone], avoided_side: int
) -> list[Point] | None:
    """
    A* from ``start`` to ``goal`` over the corners of ``zones``, or None if
    there is no way through. First legs to the ``avoided_side`` of the line to
    the goal cost ``SIDE_SWITCH_PENALTY_M`` extra.

    Whether two points can see each other is only worked out when the search
    gets to them, as most pairs never come up.
    """

    start_index = 0
    goal_index = 1
    nodes = [start, goal]
    for zone in zones:
        for corner in zone.corners():
            # Standing on a corner already: going "to" it would stall the drone
            if _distance(start, corner) < _EPSILON_M:
                continue
            # A corner swallowed by a neighbouring zone is no use
            if all(other.point_clearance_m(corner) >= 0.0 for other in zones):
                nodes.append(corner)

    cost_to = {start_index: 0.0}
    came_from: dict[int, int] = {}
    closed: set[int] = set()
    queue = [(_distance(start, goal), start_index)]

    while queue:
        _, index = heapq.heappop(queue)
        if index in closed:
            continue
        if index == goal_index:
            path = [goal]
            while index in came_from:
                index = came_from[index]
                path.append(nodes[index])
            path.reverse()
            return path
        closed.add(index)

        for neighbour_index, neighbour in enumerate(nodes):
            if neighbour_index in closed:
                continue

            cost = cost_to[index] + _distance(nodes[index], neighbour)
            if (
                index == start_index
                and avoided_side != 0
                and _side(start, goal, neighbour) == avoided_side
            ):
                cost += SIDE_SWITCH_PENALTY_M
            if cost >= cost_to.get(neighbour_index, math.inf):
                continue
            if not _is_clear(nodes[index], neighbour, zones):
                continue

            cost_to[neighbour_index] = cost
            came_from[neighbour_index] = index
            heapq.heappush(
                queue, (cost + _distance(neighbour, goal), neighbour_index)
            )

    return None


class VisibilityGraphPlanner:
    """
    Picks the next point to fly to on the way to a destination, taking the
    shortest way around keep-away zones.

    Remembers which side it last went round on so that an obstacle dead ahead
    does not make it flip between going left and going right.
    """

    def __init__(self) -> None:
        self._last_side = 0

    def plan(
        self, position: Point, destination: Point, zones: list[KeepAwayZone]
    ) -> Plan:
        """
        ``position``, ``destination`` and ``zones`` share one ``(east, north)``
        frame in meters; so does the returned point.
        """

        if not zones:
            self._last_side = 0
            return Plan(destination, PlanStatus.DIRECT)

        # Already inside a zone: getting out comes before making progress
        deepest_zone = min(zones, key=lambda zone: zone.point_clearance_m(position))
        if deepest_zone.point_clearance_m(position) < 0.0:
            escape_bearing = deepest_zone.escape_bearing_deg(position)
            return Plan(
                _project(position, escape_bearing, ESCAPE_DISTANCE_M),
                PlanStatus.ESCAPE,
            )

        # Destination inside a zone: wait as close to it as allowed until the
        # obstacle moves on
        goal = destination
        deepest_zone = min(zones, key=lambda zone: zone.point_clearance_m(goal))
        if deepest_zone.point_clearance_m(goal) < 0.0:
            goal = deepest_zone.pushed_out(goal)
            if any(zone.point_clearance_m(goal) < 0.0 for zone in zones):
                return Plan(position, PlanStatus.HOLD)

        if _is_clear(position, goal, zones):
            self._last_side = 0
            if goal == destination:
                return Plan(destination, PlanStatus.DIRECT)
            return Plan(goal, PlanStatus.DETOUR)

        path = _shortest_path(position, goal, zones, avoided_side=-self._last_side)
        if path is None:
            return Plan(position, PlanStatus.HOLD)

        next_point = path[1]
        self._last_side = _side(position, goal, next_point)

        # Aim past the corner along the same line so the drone carries its
        # speed through; by the time it gets there the next leg has opened up
        if len(path) > 2:
            distance_m = _distance(position, next_point)
            if _EPSILON_M < distance_m < DETOUR_SETPOINT_LEAD_M:
                lead_point = _project(
                    position,
                    _bearing_deg(position, next_point),
                    DETOUR_SETPOINT_LEAD_M,
                )
                if _is_clear(position, lead_point, zones):
                    next_point = lead_point

        return Plan(next_point, PlanStatus.DETOUR)
