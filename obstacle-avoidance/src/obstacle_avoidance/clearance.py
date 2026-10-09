"""Obstacle-clearance interface and a circular-obstacle implementation."""

from __future__ import annotations

import math
from typing import Protocol

from .geometry import point_segment_distance
from .models import CircleObstacle, ObstacleSnapshot, Point2D


class ClearanceField(Protocol):
    """The only geometry service required by the planner search."""

    def point_clearance(self, point: Point2D) -> float:
        """Return signed metres to the nearest obstacle boundary."""

    def segment_clearance(self, start: Point2D, end: Point2D) -> float:
        """Return minimum signed clearance along a closed segment."""


class CircularObstacleField:
    """Exact clearance against a fixed collection of circles."""

    def __init__(self, obstacles: tuple[CircleObstacle, ...]) -> None:
        self._obstacles = obstacles

    @classmethod
    def from_snapshot(cls, snapshot: ObstacleSnapshot) -> CircularObstacleField:
        return cls(snapshot.obstacles)

    def point_clearance(self, point: Point2D) -> float:
        if not self._obstacles:
            return math.inf
        return min(
            math.hypot(point.x - item.center.x, point.y - item.center.y) - item.radius_m
            for item in self._obstacles
        )

    def segment_clearance(self, start: Point2D, end: Point2D) -> float:
        if not self._obstacles:
            return math.inf
        return min(
            point_segment_distance(item.center, start, end) - item.radius_m
            for item in self._obstacles
        )
