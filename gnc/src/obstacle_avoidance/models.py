"""Data contracts: the GPS-level avoidance API and the local 2D planner."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from utils.src.types import Coordinate


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
        """Return a fail-safe hold result."""

        return cls(
            status=PlanStatus.NO_PATH,
            waypoint=None,
            second_waypoint=None,
            heading_rad=None,
            minimum_clearance_m=None,
            reason=reason,
        )


@dataclass(frozen=True, slots=True)
class TrafficAircraft:
    """One other aircraft to avoid, as reported by AEAC traffic."""

    aircraft_index: int
    lat: float
    lon: float
    altitude_agl_m: float
    speed_mps: float
    heading_deg: float  # clockwise from true north
    horizontal_keep_away_m: float
    vertical_keep_away_m: float


@dataclass(frozen=True, slots=True)
class AvoidanceRequest:
    """One control cycle: where we are, where we are going, what is around."""

    position: Coordinate  # alt is metres above the takeoff point
    goal: Coordinate
    traffic: tuple[TrafficAircraft, ...]
    traffic_age_s: float  # how long ago ``traffic`` was sampled


@dataclass(frozen=True, slots=True)
class AvoidancePlan:
    """The velocity to fly this cycle, or a zero-velocity hold with a reason."""

    status: PlanStatus
    reason: str | None
    east_mps: float
    north_mps: float
    up_mps: float
    next_waypoint: Coordinate | None
    goal_distance_m: float | None
    goal_reached: bool
    minimum_clearance_m: float | None
    obstacle_count: int

    @classmethod
    def hold(
        cls,
        reason: str,
        *,
        goal_distance_m: float | None = None,
        obstacle_count: int = 0,
    ) -> AvoidancePlan:
        return cls(
            status=PlanStatus.NO_PATH,
            reason=reason,
            east_mps=0.0,
            north_mps=0.0,
            up_mps=0.0,
            next_waypoint=None,
            goal_distance_m=goal_distance_m,
            goal_reached=False,
            minimum_clearance_m=None,
            obstacle_count=obstacle_count,
        )
