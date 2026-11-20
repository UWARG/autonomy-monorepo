"""Orchestration: GPS position, goal and traffic in; a velocity out."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .clearance import CircularObstacleField
from .models import (
    AvoidancePlan,
    AvoidanceRequest,
    ObstacleSnapshot,
    PlanRequest,
    PlanStatus,
    Point2D,
    TrafficAircraft,
)
from .planner import BendyRuler2D, PlannerConfig
from .traffic import live_traffic_buffer_m, traffic_to_obstacles


@dataclass(frozen=True, slots=True)
class AvoidanceConfig:
    # Planning output only. The position-only MAVROS setpoint is flown at the
    # FCU's configured WP_SPD/WPNAV_SPEED.
    horizontal_speed_mps: float = 2.0
    vertical_speed_mps: float = 1.0
    goal_tolerance_m: float = 1.0
    prediction_horizon_s: float = 3.0
    maximum_prediction_step_m: float = 1.0
    maximum_predicted_obstacles: int = 512
    crossing_yield_horizon_s: float = 20.0
    # Fail closed after a missed 1 Hz traffic snapshot.
    planner: PlannerConfig = field(
        default_factory=lambda: PlannerConfig(map_freshness_s=1.5)
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
            raise ValueError(
                "speeds, tolerances and prediction limits must be positive"
            )
        if self.prediction_horizon_s < 0.0:
            raise ValueError("prediction horizon must be non-negative")
        if (
            not math.isfinite(self.crossing_yield_horizon_s)
            or self.crossing_yield_horizon_s <= 0.0
        ):
            raise ValueError("crossing yield horizon must be positive")


def _valid_coordinate(point: Coordinate) -> bool:
    return (
        all(math.isfinite(value) for value in (point.lat, point.lon, point.alt))
        and -90.0 <= point.lat <= 90.0
        and -180.0 <= point.lon <= 180.0
    )


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
                all_altitudes=True,
                live_buffer=True,
                own_speed_mps=settings.horizontal_speed_mps,
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
        if result.status is PlanStatus.NO_PATH or result.waypoint is None:
            hold_clearance_m = CircularObstacleField(obstacles).point_clearance(
                Point2D(0.0, 0.0)
            )
            reason = (
                "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED"
                if hold_clearance_m < settings.planner.clearance_margin_m
                else result.reason.value
                if result.reason is not None
                else "NO_PATH"
            )
            return AvoidancePlan.hold(
                reason,
                goal_distance_m=goal_distance_m,
                obstacle_count=len(obstacles),
            )

        crossing_reason = self._crossing_risk(
            request.traffic,
            result.waypoint.x,
            result.waypoint.y,
            age_s,
            position.lat,
            position.lon,
        )
        if crossing_reason is not None:
            self.reset()
            return AvoidancePlan.hold(
                crossing_reason,
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

    def _crossing_risk(
        self,
        traffic: tuple[TrafficAircraft, ...],
        goal_east_m: float,
        goal_north_m: float,
        age_s: float,
        own_lat: float,
        own_lon: float,
    ) -> str | None:
        """Yield before entering a moving crossing path; alert if holding is unsafe."""

        distance_m = math.hypot(goal_east_m, goal_north_m)
        if distance_m <= self.config.goal_tolerance_m:
            return None
        own_speed = self.config.horizontal_speed_mps
        own_east_mps = own_speed * goal_east_m / distance_m
        own_north_mps = own_speed * goal_north_m / distance_m
        horizon_s = min(self.config.crossing_yield_horizon_s, distance_m / own_speed)
        should_yield = False
        for aircraft in traffic:
            if aircraft.speed_mps <= 0.1:
                continue
            heading_rad = math.radians(aircraft.heading_deg)
            other_east_mps = aircraft.speed_mps * math.sin(heading_rad)
            other_north_mps = aircraft.speed_mps * math.cos(heading_rad)
            along_mps = (
                other_east_mps * goal_east_m + other_north_mps * goal_north_m
            ) / distance_m
            if abs(along_mps) >= 0.5 * aircraft.speed_mps:
                continue
            east_m, north_m = east_north_coordinate_offset_m(
                own_lat, own_lon, aircraft.lat, aircraft.lon
            )
            east_m += other_east_mps * age_s
            north_m += other_north_mps * age_s
            relative_east_mps = other_east_mps - own_east_mps
            relative_north_mps = other_north_mps - own_north_mps
            relative_speed_sq = relative_east_mps**2 + relative_north_mps**2
            if relative_speed_sq <= 1e-9:
                continue
            closest_s = max(
                0.0,
                min(
                    horizon_s,
                    -(east_m * relative_east_mps + north_m * relative_north_mps)
                    / relative_speed_sq,
                ),
            )
            closest_m = math.hypot(
                east_m + relative_east_mps * closest_s,
                north_m + relative_north_mps * closest_s,
            )
            radius_m = (
                aircraft.horizontal_keep_away_m
                + live_traffic_buffer_m(aircraft.speed_mps, age_s, own_speed)
                + self.config.planner.clearance_margin_m
            )
            if closest_m >= radius_m:
                continue
            hold_closest_s = max(
                0.0,
                min(
                    horizon_s,
                    -(east_m * other_east_mps + north_m * other_north_mps)
                    / aircraft.speed_mps**2,
                ),
            )
            hold_clearance_m = math.hypot(
                east_m + other_east_mps * hold_closest_s,
                north_m + other_north_mps * hold_closest_s,
            )
            if hold_clearance_m < radius_m:
                return "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED"
            should_yield = True
        return "YIELD_CROSSING" if should_yield else None
