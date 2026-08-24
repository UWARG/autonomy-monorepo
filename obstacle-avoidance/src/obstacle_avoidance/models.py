"""Sensor-neutral data contracts for the 2D avoidance planner."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True, slots=True)
class Point2D:
    """A point in an arbitrary, consistent local Cartesian frame."""

    x: float
    y: float


@dataclass(frozen=True, slots=True)
class CircleObstacle:
    """A conservative circular obstacle in the planning plane."""

    center: Point2D
    radius_m: float

    def __post_init__(self) -> None:
        if self.radius_m < 0.0:
            raise ValueError("obstacle radius must be non-negative")


@dataclass(frozen=True, slots=True)
class ObstacleSnapshot:
    """One obstacle-map sample and its monotonic capture time."""

    obstacles: tuple[CircleObstacle, ...]
    timestamp_s: float
    healthy: bool = True


class PlanStatus(str, Enum):
    PATH_FOUND = "PATH_FOUND"
    NO_PATH = "NO_PATH"


class NoPathReason(str, Enum):
    STALE_MAP = "STALE_MAP"
    UNHEALTHY_MAP = "UNHEALTHY_MAP"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True, slots=True)
class PlanRequest:
    start: Point2D
    goal: Point2D
    obstacles: ObstacleSnapshot
    now_s: float


@dataclass(frozen=True, slots=True)
class PlanResult:
    status: PlanStatus
    waypoint: Point2D | None
    second_waypoint: Point2D | None
    heading_rad: float | None
    minimum_clearance_m: float | None
    reason: NoPathReason | None = None

    @classmethod
    def no_path(cls, reason: NoPathReason) -> PlanResult:
        """Return the fail-safe result integrations must translate to hold."""

        return cls(
            status=PlanStatus.NO_PATH,
            waypoint=None,
            second_waypoint=None,
            heading_rad=None,
            minimum_clearance_m=None,
            reason=reason,
        )
