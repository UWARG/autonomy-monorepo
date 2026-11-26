"""Orchestration: GPS position, goal and traffic in; a velocity out."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

from shapely.geometry import LineString, Point, Polygon

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

from .clearance import CircularObstacleField
from .geometry import distance, heading, project
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
    flight_boundary: tuple[Coordinate, ...] | None = None
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
        if self.flight_boundary is not None:
            if len(self.flight_boundary) < 3:
                raise ValueError("flight boundary must have at least 3 vertices")
            if any(
                not (
                    math.isfinite(vertex.lat)
                    and math.isfinite(vertex.lon)
                    and -90.0 <= vertex.lat <= 90.0
                    and -180.0 <= vertex.lon <= 180.0
                )
                for vertex in self.flight_boundary
            ):
                raise ValueError("flight boundary contains an invalid GPS vertex")
            boundary = Polygon(
                [(vertex.lon, vertex.lat) for vertex in self.flight_boundary]
            )
            if not boundary.is_valid or boundary.area <= 0.0:
                raise ValueError("flight boundary must be a valid, non-degenerate polygon")


def _valid_coordinate(point: Coordinate) -> bool:
    return (
        all(math.isfinite(value) for value in (point.lat, point.lon, point.alt))
        and -90.0 <= point.lat <= 90.0
        and -180.0 <= point.lon <= 180.0
    )


@dataclass(frozen=True, slots=True)
class _MovingTraffic:
    position: Point2D
    east_mps: float
    north_mps: float
    radius_m: float


def _flight_boundary_segment_is_safe(
    start: Point2D, end: Point2D, boundary: Polygon
) -> bool:
    segment = LineString(((start.x, start.y), (end.x, end.y)))
    start_point = Point(start.x, start.y)
    end_point = Point(end.x, end.y)
    if boundary.covers(start_point):
        return boundary.covers(segment)

    intersection = segment.intersection(boundary)
    if boundary.covers(end_point):
        if intersection.geom_type != "LineString" or intersection.is_empty:
            return False
        last_x, last_y = intersection.coords[-1]
        return math.hypot(last_x - end.x, last_y - end.y) <= 1e-6

    return intersection.is_empty and (
        boundary.distance(end_point) < boundary.distance(start_point) - 1e-6
    )


def _moving_traffic(
    traffic: tuple[TrafficAircraft, ...],
    *,
    origin_lat: float,
    origin_lon: float,
    age_s: float,
    own_speed_mps: float,
    clearance_margin_m: float,
) -> tuple[_MovingTraffic, ...]:
    motions = []
    for aircraft in traffic:
        east_m, north_m = east_north_coordinate_offset_m(
            origin_lat, origin_lon, aircraft.lat, aircraft.lon
        )
        heading_rad = math.radians(aircraft.heading_deg)
        east_mps = aircraft.speed_mps * math.sin(heading_rad)
        north_mps = aircraft.speed_mps * math.cos(heading_rad)
        motions.append(
            _MovingTraffic(
                position=Point2D(
                    east_m + east_mps * age_s,
                    north_m + north_mps * age_s,
                ),
                east_mps=east_mps,
                north_mps=north_mps,
                radius_m=(
                    aircraft.horizontal_keep_away_m
                    + live_traffic_buffer_m(
                        aircraft.speed_mps, age_s, own_speed_mps
                    )
                    + clearance_margin_m
                ),
            )
        )
    return tuple(motions)


def _closest_approach_m(
    own_position: Point2D,
    own_east_mps: float,
    own_north_mps: float,
    traffic: _MovingTraffic,
    horizon_s: float,
) -> float:
    relative_east_m = traffic.position.x - own_position.x
    relative_north_m = traffic.position.y - own_position.y
    relative_east_mps = traffic.east_mps - own_east_mps
    relative_north_mps = traffic.north_mps - own_north_mps
    relative_speed_sq = relative_east_mps**2 + relative_north_mps**2
    if relative_speed_sq <= 1e-9:
        closest_s = 0.0
    else:
        closest_s = max(
            0.0,
            min(
                horizon_s,
                -(
                    relative_east_m * relative_east_mps
                    + relative_north_m * relative_north_mps
                )
                / relative_speed_sq,
            ),
        )
    return math.hypot(
        relative_east_m + relative_east_mps * closest_s,
        relative_north_m + relative_north_mps * closest_s,
    )


def _segment_is_safe(
    start: Point2D,
    end: Point2D,
    traffic: tuple[_MovingTraffic, ...],
    *,
    own_speed_mps: float,
    horizon_s: float,
) -> bool:
    east_m = end.x - start.x
    north_m = end.y - start.y
    distance_m = math.hypot(east_m, north_m)
    if distance_m <= 1e-9:
        own_east_mps = own_north_mps = 0.0
        travel_s = 0.0
    else:
        own_east_mps = own_speed_mps * east_m / distance_m
        own_north_mps = own_speed_mps * north_m / distance_m
        travel_s = min(horizon_s, distance_m / own_speed_mps)
    return all(
        _closest_approach_m(
            start,
            own_east_mps,
            own_north_mps,
            aircraft,
            travel_s,
        )
        >= aircraft.radius_m
        for aircraft in traffic
    )


def _holding_is_safe(
    position: Point2D,
    traffic: tuple[_MovingTraffic, ...],
    *,
    horizon_s: float,
) -> bool:
    return all(
        _closest_approach_m(position, 0.0, 0.0, aircraft, horizon_s)
        >= aircraft.radius_m
        for aircraft in traffic
    )


def _egress_waypoint(
    start: Point2D,
    goal: Point2D,
    traffic: tuple[_MovingTraffic, ...],
    *,
    candidate_headings: tuple[float, ...],
    lookahead_m: float,
    own_speed_mps: float,
    flight_boundary_is_safe: Callable[[Point2D, Point2D], bool] | None = None,
) -> tuple[Point2D, float] | None:
    """Pick a heading that exits existing conflicts without creating new ones."""

    conflicts = tuple(
        aircraft
        for aircraft in traffic
        if distance(start, aircraft.position) < aircraft.radius_m
    )
    if not conflicts:
        return None

    step_m = lookahead_m
    candidates: list[tuple[float, float, int, Point2D, float]] = []

    for order, candidate_heading in enumerate(candidate_headings):
        waypoint = project(start, candidate_heading, step_m)
        candidate_speed_mps = min(own_speed_mps, step_m)
        own_east_mps = candidate_speed_mps * math.cos(candidate_heading)
        own_north_mps = candidate_speed_mps * math.sin(candidate_heading)
        exit_times_s: list[float] = []
        safe = True
        for aircraft in conflicts:
            relative_east_m = aircraft.position.x - start.x
            relative_north_m = aircraft.position.y - start.y
            relative_east_mps = aircraft.east_mps - own_east_mps
            relative_north_mps = aircraft.north_mps - own_north_mps
            start_distance_m = math.hypot(relative_east_m, relative_north_m)
            radial_rate = (
                relative_east_m * relative_east_mps
                + relative_north_m * relative_north_mps
            )
            relative_speed_sq = relative_east_mps**2 + relative_north_mps**2
            if radial_rate < -1e-9 or relative_speed_sq <= 1e-9:
                safe = False
                break

            discriminant = (
                radial_rate**2
                + relative_speed_sq
                * (aircraft.radius_m**2 - start_distance_m**2)
            )
            exit_time_s = (
                -radial_rate + math.sqrt(discriminant)
            ) / relative_speed_sq
            if not math.isfinite(exit_time_s):
                safe = False
                break
            exit_times_s.append(exit_time_s + 1e-6)

        if not safe:
            continue
        travel_s = max(step_m / candidate_speed_mps, *exit_times_s)
        simulated_end = project(
            start,
            candidate_heading,
            candidate_speed_mps * travel_s,
        )
        if flight_boundary_is_safe is not None and not flight_boundary_is_safe(
            start, simulated_end
        ):
            continue
        progress_m: list[float] = []
        minimum_clearance_m = math.inf

        for aircraft in traffic:
            relative_east_m = aircraft.position.x - start.x
            relative_north_m = aircraft.position.y - start.y
            relative_east_mps = aircraft.east_mps - own_east_mps
            relative_north_mps = aircraft.north_mps - own_north_mps
            start_distance_m = math.hypot(relative_east_m, relative_north_m)
            closest_m = _closest_approach_m(
                start,
                own_east_mps,
                own_north_mps,
                aircraft,
                travel_s,
            )
            minimum_clearance_m = min(
                minimum_clearance_m, closest_m - aircraft.radius_m
            )

            if start_distance_m < aircraft.radius_m:
                radial_rate = (
                    relative_east_m * relative_east_mps
                    + relative_north_m * relative_north_mps
                )
                end_distance_m = math.hypot(
                    relative_east_m + relative_east_mps * travel_s,
                    relative_north_m + relative_north_mps * travel_s,
                )
                if (
                    radial_rate < -1e-9
                    or end_distance_m < aircraft.radius_m
                    or end_distance_m <= start_distance_m
                ):
                    safe = False
                    break
                progress_m.append(end_distance_m - start_distance_m)
            elif closest_m < aircraft.radius_m:
                safe = False
                break

        if safe and progress_m:
            candidates.append(
                (
                    min(progress_m),
                    -distance(waypoint, goal),
                    -order,
                    waypoint,
                    minimum_clearance_m,
                )
            )

    if not candidates:
        return None
    selected = max(candidates, key=lambda item: item[:3])
    return selected[3], selected[4]


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
        flight_boundary = None
        flight_boundary_is_safe = None
        if settings.flight_boundary is not None:
            flight_boundary = Polygon(
                [
                    east_north_coordinate_offset_m(
                        position.lat,
                        position.lon,
                        vertex.lat,
                        vertex.lon,
                    )
                    for vertex in settings.flight_boundary
                ]
            )

            def flight_boundary_is_safe(
                start: Point2D, end: Point2D
            ) -> bool:
                return _flight_boundary_segment_is_safe(
                    start, end, flight_boundary
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

        moving_traffic = _moving_traffic(
            request.traffic,
            origin_lat=position.lat,
            origin_lon=position.lon,
            age_s=age_s,
            own_speed_mps=settings.horizontal_speed_mps,
            clearance_margin_m=settings.planner.clearance_margin_m,
        )
        dynamic_conflict_seen = False

        def dynamic_segment_is_safe(start: Point2D, end: Point2D) -> bool:
            nonlocal dynamic_conflict_seen
            safe = _segment_is_safe(
                start,
                end,
                moving_traffic,
                own_speed_mps=settings.horizontal_speed_mps,
                horizon_s=settings.crossing_yield_horizon_s,
            )
            dynamic_conflict_seen = dynamic_conflict_seen or not safe
            return safe

        field = CircularObstacleField(obstacles)
        start = Point2D(0.0, 0.0)
        current_conflicts = tuple(
            aircraft
            for aircraft in moving_traffic
            if distance(start, aircraft.position) < aircraft.radius_m
        )
        if current_conflicts:
            egress = _egress_waypoint(
                start,
                Point2D(goal_east_m, goal_north_m),
                moving_traffic,
                candidate_headings=self._planner.candidate_headings(
                    heading(start, Point2D(goal_east_m, goal_north_m))
                ),
                lookahead_m=settings.planner.first_lookahead_m,
                own_speed_mps=settings.horizontal_speed_mps,
                flight_boundary_is_safe=flight_boundary_is_safe,
            )
            self._planner.reset()
            if egress is None:
                return AvoidancePlan.hold(
                    "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED",
                    goal_distance_m=goal_distance_m,
                    obstacle_count=len(obstacles),
                )

            waypoint, minimum_clearance_m = egress
            step_m = math.hypot(waypoint.x, waypoint.y)
            speed_mps = min(settings.horizontal_speed_mps, step_m)
            east_mps = speed_mps * waypoint.x / step_m
            north_mps = speed_mps * waypoint.y / step_m
            up_mps = max(
                -settings.vertical_speed_mps,
                min(settings.vertical_speed_mps, altitude_error_m),
            )
            return AvoidancePlan(
                status=PlanStatus.PATH_FOUND,
                reason="EGRESSING_CONFLICT",
                east_mps=east_mps,
                north_mps=north_mps,
                up_mps=up_mps,
                next_waypoint=enu_to_coordinate(
                    position, waypoint.x, waypoint.y, goal.alt
                ),
                goal_distance_m=goal_distance_m,
                goal_reached=False,
                minimum_clearance_m=minimum_clearance_m,
                obstacle_count=len(obstacles),
            )

        result = self._planner.plan(
            PlanRequest(
                start=start,
                goal=Point2D(goal_east_m, goal_north_m),
                obstacles=ObstacleSnapshot(obstacles, timestamp_s=-age_s),
                now_s=0.0,
            ),
            dynamic_segment_is_safe=dynamic_segment_is_safe,
            flight_boundary_is_safe=flight_boundary_is_safe,
        )
        if result.status is PlanStatus.NO_PATH or result.waypoint is None:
            hold_clearance_m = field.point_clearance(start)
            hold_is_safe = (
                hold_clearance_m >= settings.planner.clearance_margin_m
                and _holding_is_safe(
                    start,
                    moving_traffic,
                    horizon_s=settings.crossing_yield_horizon_s,
                )
            )
            reason = (
                "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED"
                if not hold_is_safe
                else "YIELD_CROSSING"
                if dynamic_conflict_seen
                else result.reason.value
                if result.reason is not None
                else "NO_PATH"
            )
            return AvoidancePlan.hold(
                reason,
                goal_distance_m=goal_distance_m,
                obstacle_count=len(obstacles),
            )
        waypoint = result.waypoint

#        crossing_reason = self._crossing_risk(
#            request.traffic,
#            result.waypoint.x,
#            result.waypoint.y,
#            age_s,
#            position.lat,
#            position.lon,
#        )
#        if crossing_reason is not None:
#            self.reset()
#            return AvoidancePlan.hold(
#                crossing_reason,
#                goal_distance_m=goal_distance_m,
#                obstacle_count=len(obstacles),
#            )

        reached = goal_distance_m <= settings.goal_tolerance_m
        if reached and not _holding_is_safe(
            start,
            moving_traffic,
            horizon_s=settings.crossing_yield_horizon_s,
        ):
            self.reset()
            return AvoidancePlan.hold(
                "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED",
                goal_distance_m=goal_distance_m,
                obstacle_count=len(obstacles),
            )
        east_mps = north_mps = up_mps = 0.0
        if not reached:
            step_m = math.hypot(waypoint.x, waypoint.y)
            if step_m > 0.0:
                speed_mps = min(settings.horizontal_speed_mps, step_m)
                east_mps = speed_mps * waypoint.x / step_m
                north_mps = speed_mps * waypoint.y / step_m
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
                position, waypoint.x, waypoint.y, goal.alt
            ),
            goal_distance_m=goal_distance_m,
            goal_reached=reached,
            minimum_clearance_m=result.minimum_clearance_m,
            obstacle_count=len(obstacles),
        )