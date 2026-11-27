"""Orchestration: GPS position, goal and traffic in; a velocity out."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .clearance import CircularObstacleField
from .geometry import angular_distance, heading, project
from .models import (
    AvoidancePlan,
    AvoidanceRequest,
    CircleObstacle,
    NoPathReason,
    ObstacleSnapshot,
    PlanRequest,
    PlanStatus,
    Point2D,
)
from .planner import BendyRuler2D, PlannerConfig
from .traffic import traffic_to_obstacles


@dataclass(frozen=True, slots=True)
class AvoidanceConfig:
    horizontal_speed_mps: float = 2.0
    vertical_speed_mps: float = 1.0
    goal_tolerance_m: float = 1.0
    prediction_horizon_s: float = 3.0
    maximum_prediction_step_m: float = 1.0
    maximum_predicted_obstacles: int = 512
    # AEAC traffic arrives at 1 Hz, so tolerate a couple of missed updates.
    planner: PlannerConfig = field(
        default_factory=lambda: PlannerConfig(map_freshness_s=2.5)
    )

    def __post_init__(self) -> None:
        positive = (
            self.horizontal_speed_mps,
            self.vertical_speed_mps,
            self.goal_tolerance_m,
            self.maximum_prediction_step_m,
            self.maximum_predicted_obstacles,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("speeds, tolerances and prediction limits must be positive")
        if self.prediction_horizon_s < 0.0:
            raise ValueError("prediction horizon must be non-negative")


def _valid_coordinate(point: Coordinate) -> bool:
    return (
        all(math.isfinite(value) for value in (point.lat, point.lon, point.alt))
        and -90.0 <= point.lat <= 90.0
        and -180.0 <= point.lon <= 180.0
    )


# AvoidancePlan.reason while backing away from traffic with no path to the goal.
ESCAPE_REASON = "ESCAPING_TRAFFIC"


class ObstacleAvoider:
    """
    Steers toward a goal around traffic, one control cycle at a time.

    Keeps the planner's heading hysteresis between calls, so use one instance
    per flight leg and call :meth:`reset` when control is interrupted. Every
    failure (stale or invalid traffic, no safe path) is a zero-velocity hold.
    """

    def __init__(self, config: AvoidanceConfig | None = None) -> None:
        self.config = config or AvoidanceConfig()
        self._planner = BendyRuler2D(self.config.planner)

    def reset(self) -> None:
        self._planner.reset()

    def plan(self, request: AvoidanceRequest) -> AvoidancePlan:
        settings = self.config
        position, goal = request.position, request.goal
        if not (_valid_coordinate(position) and _valid_coordinate(goal)):
            self.reset()
            return AvoidancePlan.hold("INVALID_POSITION")
        age_s = request.traffic_age_s
        if not 0.0 <= age_s <= settings.planner.map_freshness_s:
            self.reset()
            return AvoidancePlan.hold("STALE_TRAFFIC")

        # Local ENU frame with the vehicle at the origin.
        goal_east_m, goal_north_m = east_north_coordinate_offset_m(
            position.lat, position.lon, goal.lat, goal.lon
        )
        altitude_error_m = goal.alt - position.alt
        goal_distance_m = math.sqrt(
            goal_east_m**2 + goal_north_m**2 + altitude_error_m**2
        )

        try:
            obstacles = traffic_to_obstacles(
                request.traffic,
                origin_lat=position.lat,
                origin_lon=position.lon,
                altitude_band_m=(position.alt, goal.alt),
                age_s=age_s,
                prediction_horizon_s=settings.prediction_horizon_s,
                maximum_step_m=settings.maximum_prediction_step_m,
                maximum_obstacles=settings.maximum_predicted_obstacles,
            )
        except ValueError as error:
            self.reset()
            return AvoidancePlan.hold(str(error), goal_distance_m=goal_distance_m)

        result = self._planner.plan(
            PlanRequest(
                start=Point2D(0.0, 0.0),
                goal=Point2D(goal_east_m, goal_north_m),
                obstacles=ObstacleSnapshot(obstacles, timestamp_s=-age_s),
                now_s=0.0,
            )
        )
        if result.reason is NoPathReason.BLOCKED:
            # Holding still is not safe against moving traffic: an aircraft
            # flying at us blocks every forward heading and then flies through
            # the hovering drone. Back off to the clearest reachable point.
            escape = self._escape(
                position, Point2D(goal_east_m, goal_north_m), obstacles, goal_distance_m
            )
            if escape is not None:
                return escape
        if result.status is PlanStatus.NO_PATH or result.waypoint is None:
            return AvoidancePlan.hold(
                result.reason.value if result.reason is not None else "NO_PATH",
                goal_distance_m=goal_distance_m,
                obstacle_count=len(obstacles),
            )

        reached = goal_distance_m <= settings.goal_tolerance_m
        east_mps = north_mps = up_mps = 0.0
        if not reached:
            step_m = math.hypot(result.waypoint.x, result.waypoint.y)
            if step_m > 0.0:
                speed_mps = min(settings.horizontal_speed_mps, step_m)
                east_mps = speed_mps * result.waypoint.x / step_m
                north_mps = speed_mps * result.waypoint.y / step_m
            up_mps = max(
                -settings.vertical_speed_mps,
                min(settings.vertical_speed_mps, altitude_error_m),
            )
        return AvoidancePlan(
            status=PlanStatus.PATH_FOUND,
            reason=None,
            east_mps=east_mps,
            north_mps=north_mps,
            up_mps=up_mps,
            next_waypoint=enu_to_coordinate(
                position, result.waypoint.x, result.waypoint.y, goal.alt
            ),
            goal_distance_m=goal_distance_m,
            goal_reached=reached,
            minimum_clearance_m=result.minimum_clearance_m,
            obstacle_count=len(obstacles),
        )

    def _escape(
        self,
        position: Coordinate,
        goal: Point2D,
        obstacles: tuple[CircleObstacle, ...],
        goal_distance_m: float,
    ) -> AvoidancePlan | None:
        """
        A step to the point with the most clearance, or None to hold.

        Searches all round (backwards too) at the planner's first lookahead.
        A step qualifies only if nothing along it is closer to any keep-away
        than the drone is now, and it ends clearer than here; ties go to the
        heading nearest the goal. Altitude is held.
        """

        planner = self.config.planner
        field = CircularObstacleField(obstacles)
        start = Point2D(0.0, 0.0)
        current_m = field.point_clearance(start)
        goal_heading = heading(start, goal) if goal != start else 0.0
        step_rad = math.radians(planner.candidate_step_deg)

        best: tuple[float, float, Point2D] | None = None
        for index in range(int(round(2 * math.pi / step_rad))):
            candidate_heading = goal_heading + index * step_rad
            end = project(start, candidate_heading, planner.first_lookahead_m)
            if field.segment_clearance(start, end) < current_m - 1e-9:
                continue
            end_clearance_m = field.point_clearance(end)
            if end_clearance_m <= current_m:
                continue
            key = (-end_clearance_m, angular_distance(candidate_heading, goal_heading), end)
            if best is None or key[:2] < best[:2]:
                best = key
        if best is None:
            return None

        end = best[2]
        speed_mps = min(self.config.horizontal_speed_mps, planner.first_lookahead_m)
        return AvoidancePlan(
            status=PlanStatus.PATH_FOUND,
            reason=ESCAPE_REASON,
            east_mps=speed_mps * end.x / planner.first_lookahead_m,
            north_mps=speed_mps * end.y / planner.first_lookahead_m,
            up_mps=0.0,
            next_waypoint=enu_to_coordinate(position, end.x, end.y, position.alt),
            goal_distance_m=goal_distance_m,
            goal_reached=False,
            minimum_clearance_m=field.segment_clearance(start, end),
            obstacle_count=len(obstacles),
        )
