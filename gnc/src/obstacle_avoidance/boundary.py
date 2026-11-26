"""Horizontal companion-computer guard for an approved WGS84 flight polygon."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

from shapely.geometry import LineString, Point, Polygon
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m


@dataclass(frozen=True, slots=True)
class FlightBoundary:
    origin_lat: float
    origin_lon: float
    polygon: Polygon
    inner: Polygon

    @classmethod
    def from_json(cls, raw: str, *, margin_m: float = 5.0) -> FlightBoundary:
        """Parse ``[[lat, lon], ...]``; a missing/invalid polygon is not usable."""

        try:
            vertices = json.loads(raw)
            if not isinstance(vertices, list) or len(vertices) < 3:
                raise ValueError
            pairs = []
            for vertex in vertices:
                if not isinstance(vertex, list) or len(vertex) != 2:
                    raise ValueError
                lat, lon = map(float, vertex)
                if not (
                    math.isfinite(lat)
                    and math.isfinite(lon)
                    and -90 <= lat <= 90
                    and -180 <= lon <= 180
                ):
                    raise ValueError
                pairs.append((lat, lon))
            if not math.isfinite(margin_m) or margin_m <= 0:
                raise ValueError
            origin_lat, origin_lon = pairs[0]
            local = [
                east_north_coordinate_offset_m(origin_lat, origin_lon, lat, lon)
                for lat, lon in pairs
            ]
            polygon = Polygon(local)
            if not polygon.is_valid or polygon.area <= 0:
                raise ValueError
            inner = polygon.buffer(-margin_m)
            if inner.is_empty or inner.geom_type != "Polygon":
                raise ValueError
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("INVALID_FLIGHT_BOUNDARY") from error
        return cls(origin_lat, origin_lon, polygon, inner)

    def _local(self, point: Coordinate) -> tuple[float, float]:
        if not (
            math.isfinite(point.lat)
            and math.isfinite(point.lon)
            and -90 <= point.lat <= 90
            and -180 <= point.lon <= 180
        ):
            raise ValueError("invalid position")
        return east_north_coordinate_offset_m(
            self.origin_lat, self.origin_lon, point.lat, point.lon
        )

    def allows_point(self, point: Coordinate) -> bool:
        try:
            return self.inner.covers(Point(self._local(point)))
        except ValueError:
            return False

    def allows_point_with_margin(self, point: Coordinate, margin_m: float) -> bool:
        """Keep the entire waypoint acceptance disc inside the inset fence."""

        if not math.isfinite(margin_m) or margin_m < 0:
            return False
        try:
            return self.inner.buffer(-margin_m).covers(Point(self._local(point)))
        except ValueError:
            return False

    def allows_segment(self, start: Coordinate, target: Coordinate) -> bool:
        try:
            return self.inner.covers(
                LineString((self._local(start), self._local(target)))
            )
        except ValueError:
            return False
