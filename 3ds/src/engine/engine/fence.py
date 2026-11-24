"""
Geofence geometry for the 3DS safety supervisor and position controller.

Vertices are converted once to local east/north metres anchored at the fence
centroid, so every query is plain planar geometry in metres.
"""

from __future__ import annotations

import math

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import nearest_points

from utils.src.coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
    enu_to_coordinate,
)
from utils.src.types import Coordinate

# Log a warning this close to the fence, or this many seconds from crossing it.
WARNING_DISTANCE_M = 50.0
WARNING_TIME_S = 20.0
# Aim this far inside the fence when recovering. It must clear the warning
# band, or the drone would still be "near" at the recovery point.
RECOVERY_MARGIN_M = 75.0
# Stay clear this long before handing control back to the mission.
SAFE_HOLD_S = 2.0

# Below this speed the drone counts as not closing on the fence. ETAs beyond
# this horizon are hidden too: slow drift and velocity noise produce numbers
# in the thousands of seconds that are not actionable.
_MIN_SPEED_MPS = 0.1
_MAX_ETA_S = 60.0


class Fence:
    """A 2D polygon fence in local east/north metres."""

    def __init__(self, vertices: list[Coordinate]) -> None:
        if len(vertices) < 3:
            raise ValueError("a geofence needs at least 3 vertices")
        self.vertices = list(vertices)
        self.anchor = anchor_for_boundary(vertices)
        self._polygon = Polygon(boundary_to_enu(self.anchor, vertices))
        self._recovery_area = self._polygon.buffer(-RECOVERY_MARGIN_M)

        min_east, min_north, max_east, max_north = self._polygon.bounds
        self._ray_length_m = (
            math.hypot(max_east - min_east, max_north - min_north) + 1.0
        )

    def _point(self, position: Coordinate) -> Point:
        return Point(coordinate_to_enu(self.anchor, position))

    def contains(self, position: Coordinate) -> bool:
        """True if the position is inside the fence (edges included)."""
        return bool(self._polygon.covers(self._point(position)))

    def clearance_m(self, position: Coordinate) -> float:
        """Metres to the fence edge: positive inside, negative outside."""
        point = self._point(position)
        if self._polygon.covers(point):
            return self._polygon.exterior.distance(point)
        return -self._polygon.distance(point)

    def time_to_breach_s(
        self, position: Coordinate, velocity_enu: tuple[float, float, float]
    ) -> float:
        """
        Seconds until the fence is crossed at the current velocity.

        Returns ``0.0`` when already outside and ``math.inf`` when stationary,
        too slow, or too far from crossing to give a useful ETA.
        """
        if not self.contains(position):
            return 0.0

        east_velocity, north_velocity = velocity_enu[0], velocity_enu[1]
        speed = math.hypot(east_velocity, north_velocity)
        if speed < _MIN_SPEED_MPS:
            return math.inf

        point = self._point(position)
        endpoint = Point(
            point.x + east_velocity / speed * self._ray_length_m,
            point.y + north_velocity / speed * self._ray_length_m,
        )
        hit = self._polygon.exterior.intersection(LineString([point, endpoint]))
        if hit.is_empty:
            return math.inf
        eta_s = point.distance(hit) / speed
        return eta_s if eta_s <= _MAX_ETA_S else math.inf

    def breached(self, position: Coordinate) -> bool:
        """True once the drone is actually outside the fence."""
        return not self.contains(position)

    def near(
        self, position: Coordinate, velocity_enu: tuple[float, float, float]
    ) -> bool:
        """True when the drone is close to the fence, breached or not."""
        return (
            self.clearance_m(position) <= WARNING_DISTANCE_M
            or self.time_to_breach_s(position, velocity_enu) <= WARNING_TIME_S
        )

    def recovery_coordinate(self, position: Coordinate) -> Coordinate:
        """Nearest position inside the fence that respects the recovery margin."""
        target = nearest_points(self._recovery_area, self._point(position))[0]
        return enu_to_coordinate(self.anchor, target.x, target.y, position.alt)
