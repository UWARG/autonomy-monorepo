"""Orchestration: GPS position, goal and traffic in; a velocity out."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field, replace

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
from .patrol import BackAndForthHistory, BackAndForthPrediction
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
    # Buffer beyond the server keep-away for traffic confirmed as flying back
    # and forth. Replaces the speed-based live buffer: the reflected-motion
    # model already predicts where the aircraft will be.
    patrol_traffic_buffer_m: float = 0.0
    # Whether a confirmed patrol's circle also grows by the model's estimated
    # prediction error (endpoint, sampling and pause allowances, plus growth
    # over time). Off, the circle is keep-away + clearance margin + patrol
    # buffer around the predicted position, trusting the model exactly.
    patrol_model_uncertainty: bool = False
    # How much of each moving aircraft's recent track (seconds) forms a band
    # the drone may cross but not stop in, since the aircraft may fly back
    # along it.
    traffic_track_window_s: float = 30.0
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
            not math.isfinite(self.patrol_traffic_buffer_m)
            or self.patrol_traffic_buffer_m < 0.0
        ):
            raise ValueError("patrol traffic buffer must be non-negative")
        if (
            not math.isfinite(self.traffic_track_window_s)
            or self.traffic_track_window_s <= 0.0
        ):
            raise ValueError("traffic track window must be positive")
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
    patrol: BackAndForthPrediction | None = None
    patrol_age_s: float = 0.0


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
    patrol_predictions: dict[int, BackAndForthPrediction],
    patrol_buffer_m: float,
    patrol_model_uncertainty: bool = True,
) -> tuple[_MovingTraffic, ...]:
    motions = []
    for aircraft in traffic:
        east_m, north_m = east_north_coordinate_offset_m(
            origin_lat, origin_lon, aircraft.lat, aircraft.lon
        )
        heading_rad = math.radians(aircraft.heading_deg)
        east_mps = aircraft.speed_mps * math.sin(heading_rad)
        north_mps = aircraft.speed_mps * math.cos(heading_rad)
        patrol = patrol_predictions.get(aircraft.aircraft_index)
        if patrol is not None and not patrol_model_uncertainty:
            patrol = replace(patrol, uncertainty_m=0.0, uncertainty_growth_mps=0.0)
        position = (
            patrol.position_at(age_s)
            if patrol is not None
            else Point2D(
                east_m + east_mps * age_s,
                north_m + north_mps * age_s,
            )
        )
        motions.append(
            _MovingTraffic(
                position=position,
                east_mps=east_mps,
                north_mps=north_mps,
                radius_m=(
                    aircraft.horizontal_keep_away_m
                    + clearance_margin_m
                    + (
                        patrol_buffer_m + patrol.uncertainty_at(age_s)
                        if patrol is not None
                        else live_traffic_buffer_m(
                            aircraft.speed_mps, age_s, own_speed_mps
                        )
                    )
                ),
                patrol=patrol,
                patrol_age_s=age_s,
            )
        )
    return tuple(motions)


def _traffic_position_at(traffic: _MovingTraffic, elapsed_s: float) -> Point2D:
    if traffic.patrol is not None:
        return traffic.patrol.position_at(traffic.patrol_age_s + elapsed_s)
    return Point2D(
        traffic.position.x + traffic.east_mps * elapsed_s,
        traffic.position.y + traffic.north_mps * elapsed_s,
    )


def _traffic_radius_at(traffic: _MovingTraffic, elapsed_s: float) -> float:
    if traffic.patrol is None:
        return traffic.radius_m
    return traffic.radius_m + max(
        0.0,
        traffic.patrol.uncertainty_at(traffic.patrol_age_s + elapsed_s)
        - traffic.patrol.uncertainty_at(traffic.patrol_age_s),
    )


def _sampled_motion_is_safe(
    own_position: Point2D,
    own_east_mps: float,
    own_north_mps: float,
    travel_s: float,
    traffic: _MovingTraffic,
) -> bool:
    if traffic.patrol is None:
        raise ValueError("sampled motion requires a patrol prediction")
    relative_speed_mps = (
        math.hypot(own_east_mps, own_north_mps) + traffic.patrol.speed_mps
    )
    sample_step_s = max(
        0.05,
        min(
            0.25,
            traffic.radius_m / (2.0 * max(relative_speed_mps, 0.1)),
        ),
    )
    steps = max(1, math.ceil(travel_s / sample_step_s))
    for step in range(steps + 1):
        elapsed_s = travel_s * step / steps
        own = Point2D(
            own_position.x + own_east_mps * elapsed_s,
            own_position.y + own_north_mps * elapsed_s,
        )
        if distance(own, _traffic_position_at(traffic, elapsed_s)) < _traffic_radius_at(
            traffic, elapsed_s
        ):
            return False
    return True


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
    for aircraft in traffic:
        if aircraft.patrol is not None:
            if not _sampled_motion_is_safe(
                start,
                own_east_mps,
                own_north_mps,
                travel_s,
                aircraft,
            ):
                return False
        elif (
            _closest_approach_m(
                start,
                own_east_mps,
                own_north_mps,
                aircraft,
                travel_s,
            )
            < aircraft.radius_m
        ):
            return False
    return True


# Shortest recent track (meters) that forms a band; shorter is a hover.
_MIN_TRACK_LENGTH_M = 1.0
# How far past a band's edge a crossing or retreat aims (meters).
_BAND_EXIT_MARGIN_M = 1.0
# Time step (seconds) when checking a band crossing.
_BAND_SAMPLE_STEP_S = 0.25


@dataclass(frozen=True, slots=True)
class _TrafficTrack:
    """The band around an aircraft's recent track, which it may fly back along."""

    line: LineString
    band: Polygon
    traffic: _MovingTraffic
    speed_mps: float
    aircraft_index: int = -1


def _traffic_tracks(
    traffic: tuple[TrafficAircraft, ...],
    moving_traffic: tuple[_MovingTraffic, ...],
    recent_tracks: dict[int, tuple[Point2D, ...]],
) -> tuple[_TrafficTrack, ...]:
    tracks = []
    for aircraft, motion in zip(traffic, moving_traffic, strict=True):
        points = recent_tracks.get(aircraft.aircraft_index, ())
        if aircraft.speed_mps <= 0.1 or len(points) < 2:
            continue
        line = LineString([(point.x, point.y) for point in points])
        if line.length < _MIN_TRACK_LENGTH_M:
            continue
        tracks.append(
            _TrafficTrack(
                line=line,
                band=line.buffer(motion.radius_m),
                traffic=motion,
                speed_mps=aircraft.speed_mps,
                aircraft_index=aircraft.aircraft_index,
            )
        )
    return tuple(tracks)


def _band_exit_m(
    start: Point2D, unit_x: float, unit_y: float, from_m: float, track: _TrafficTrack
) -> float | None:
    """
    Distance along the ray from ``start`` where the stretch of band holding
    the point ``from_m`` along the ray ends, or None if it never does.
    """

    reach_m = from_m + 2.0 * track.traffic.radius_m + track.line.length
    ray = LineString(
        (
            (start.x, start.y),
            (start.x + unit_x * reach_m, start.y + unit_y * reach_m),
        )
    )
    inside = ray.intersection(track.band)
    for part in getattr(inside, "geoms", (inside,)):
        if part.is_empty or part.geom_type != "LineString":
            continue
        ends_m = sorted(
            ray.project(Point(coords)) for coords in (part.coords[0], part.coords[-1])
        )
        if ends_m[0] - 1e-6 <= from_m <= ends_m[1] + 1e-6:
            if ends_m[1] >= reach_m - 1e-6:
                return None
            return ends_m[1]
    return from_m


def _track_crossing_is_safe(
    start: Point2D,
    end: Point2D,
    track: _TrafficTrack,
    *,
    own_speed_mps: float,
    horizon_s: float,
    flight_boundary_is_safe: Callable[[Point2D, Point2D], bool] | None,
) -> bool:
    """
    A leg ending inside the band is only safe if carrying on straight clears
    the band and the whole crossing is safe: against the patrol model when
    the aircraft's pattern is confirmed, otherwise against the aircraft
    turning back along its track at any moment.
    """

    length_m = distance(start, end)
    if length_m <= 0.0:
        return True
    if not track.band.intersects(LineString(((start.x, start.y), (end.x, end.y)))):
        return True
    unit_x = (end.x - start.x) / length_m
    unit_y = (end.y - start.y) / length_m
    travel_m = length_m
    if track.line.distance(Point(end.x, end.y)) < track.traffic.radius_m:
        exit_m = _band_exit_m(start, unit_x, unit_y, length_m, track)
        if exit_m is None:
            return False
        travel_m = exit_m + _BAND_EXIT_MARGIN_M
        exit_point = Point2D(start.x + unit_x * travel_m, start.y + unit_y * travel_m)
        if flight_boundary_is_safe is not None and not flight_boundary_is_safe(
            start, exit_point
        ):
            return False
    travel_s = travel_m / own_speed_mps
    if travel_s > horizon_s:
        return False
    own_east_mps = own_speed_mps * unit_x
    own_north_mps = own_speed_mps * unit_y
    traffic = track.traffic
    if traffic.patrol is not None:
        return _sampled_motion_is_safe(
            start, own_east_mps, own_north_mps, travel_s, traffic
        )
    # Not a confirmed pattern: the aircraft may turn back along its track at
    # any time, so while inside the band, stay out of its reach.
    steps = max(1, math.ceil(travel_s / _BAND_SAMPLE_STEP_S))
    for step in range(steps + 1):
        elapsed_s = travel_s * step / steps
        own = Point2D(
            start.x + own_east_mps * elapsed_s, start.y + own_north_mps * elapsed_s
        )
        if track.line.distance(Point(own.x, own.y)) >= traffic.radius_m:
            continue
        if distance(own, traffic.position) < (
            traffic.radius_m + track.speed_mps * elapsed_s
        ):
            return False
    return True


def _track_retreat_waypoint(
    start: Point2D,
    goal: Point2D,
    tracks: tuple[_TrafficTrack, ...],
    moving_traffic: tuple[_MovingTraffic, ...],
    *,
    own_speed_mps: float,
    horizon_s: float,
    flight_boundary_is_safe: Callable[[Point2D, Point2D], bool] | None,
) -> Point2D | None:
    """
    The quickest way out of the deepest band the drone is in: straight away
    from the track. Crossing to the far side instead is only taken when the
    near side is blocked and the crossing is safe against all traffic.
    """

    here = Point(start.x, start.y)
    inside = [
        (track.traffic.radius_m - track.line.distance(here), track)
        for track in tracks
        if track.line.distance(here) < track.traffic.radius_m
    ]
    if not inside:
        return None
    depth_m, track = max(inside, key=lambda item: item[0])
    along_m = track.line.project(here)
    nearest = track.line.interpolate(along_m)
    away_x, away_y = start.x - nearest.x, start.y - nearest.y
    off_m = math.hypot(away_x, away_y)
    if off_m < 0.5:
        # On the track: step off it sideways, on the goal's side.
        before = track.line.interpolate(max(0.0, along_m - 1.0))
        after = track.line.interpolate(min(track.line.length, along_m + 1.0))
        along_x, along_y = after.x - before.x, after.y - before.y
        norm = math.hypot(along_x, along_y) or 1.0
        away_x, away_y = -along_y / norm, along_x / norm
        if away_x * (goal.x - start.x) + away_y * (goal.y - start.y) < 0.0:
            away_x, away_y = -away_x, -away_y
    else:
        away_x, away_y = away_x / off_m, away_y / off_m

    others = tuple(motion for motion in moving_traffic if motion is not track.traffic)
    options = (
        (1.0, depth_m + _BAND_EXIT_MARGIN_M, others),
        (-1.0, track.traffic.radius_m + off_m + _BAND_EXIT_MARGIN_M, moving_traffic),
    )
    for sign, step_m, checked_traffic in options:
        waypoint = Point2D(
            start.x + sign * away_x * step_m, start.y + sign * away_y * step_m
        )
        if flight_boundary_is_safe is not None and not flight_boundary_is_safe(
            start, waypoint
        ):
            continue
        if _segment_is_safe(
            start,
            waypoint,
            checked_traffic,
            own_speed_mps=own_speed_mps,
            horizon_s=horizon_s,
        ):
            return waypoint
    return None


# Patrol crossing search.
# Extra distance (meters) beyond the patrol's circle radius for the staging
# point before a crossing and the exit point after it.
_CROSSING_MARGIN_M = 2.0
# Spacing (meters) of candidate crossing points along the patrol line.
_CROSSING_STEP_M = 2.0
# Candidates extend this far (meters) past each end of the patrol line, plus
# the staging offset, so going around an end is considered too.
_CROSSING_END_REACH_M = 10.0
# How far ahead (seconds) to look for a gap, and the time step for it.
_CROSSING_LOOKAHEAD_S = 60.0
_CROSSING_STEP_S = 0.1
# Within this distance (meters) of the staging or exit point counts as there.
_CROSSING_ARRIVAL_M = 1.5
# Give up on a chosen crossing after this long (seconds) and choose again.
_CROSSING_TIMEOUT_S = 120.0


@dataclass(slots=True)
class _PatrolCrossing:
    """A chosen place to cross a patrol line, kept across control cycles."""

    aircraft_index: int
    staging: Coordinate
    exit: Coordinate
    chosen_at_s: float
    crossing: bool = False


@dataclass(frozen=True, slots=True)
class _CrossingOption:
    cost_s: float
    along_m: float
    staging: Point2D
    exit: Point2D


def _patrol_band(patrol: BackAndForthPrediction, radius_m: float) -> Polygon:
    """The area a confirmed patrol sweeps: its line, buffered by its radius."""

    low = patrol.point_along(patrol.lower_m)
    high = patrol.point_along(patrol.upper_m)
    return LineString(((low.x, low.y), (high.x, high.y))).buffer(radius_m)


class _PatrolCrossingSearch:
    """
    Where along a confirmed patrol line to cross, and when.

    A crossing at ``along_m`` flies straight across the line, from a staging
    point just outside the patrol's circle on the drone's side to an exit
    point just outside it on the far side. It is feasible when, after flying
    to the staging point, the predicted aircraft stays at least the circle
    radius along the line from the crossing for the whole crossing. That is
    stricter than the straight-line distance, so a crossing found here also
    passes the exact check made when it starts.
    """

    def __init__(
        self,
        start: Point2D,
        goal: Point2D,
        patrol: BackAndForthPrediction,
        radius_m: float,
        age_s: float,
        own_speed_mps: float,
    ) -> None:
        self._start = start
        self._goal = goal
        self._patrol = patrol
        self._radius_m = radius_m
        self._own_speed_mps = own_speed_mps
        normal_x, normal_y = -patrol.axis_y, patrol.axis_x
        side = (start.x - patrol.center.x) * normal_x + (
            start.y - patrol.center.y
        ) * normal_y
        if abs(side) < 1e-6:
            side = -(
                (goal.x - patrol.center.x) * normal_x
                + (goal.y - patrol.center.y) * normal_y
            )
        sign = 1.0 if side >= 0.0 else -1.0
        # Unit normal pointing to the drone's side of the line.
        self._toward_x, self._toward_y = sign * normal_x, sign * normal_y
        self._offset_m = radius_m + _CROSSING_MARGIN_M
        self._crossing_s = 2.0 * self._offset_m / own_speed_mps
        self._window_steps = math.ceil(self._crossing_s / _CROSSING_STEP_S)
        steps = math.ceil(_CROSSING_LOOKAHEAD_S / _CROSSING_STEP_S)
        self._along_m = [
            patrol.along_at(age_s + step * _CROSSING_STEP_S)
            for step in range(steps + 1)
        ]

    def along_of(self, point: Point2D) -> float:
        patrol = self._patrol
        return (point.x - patrol.center.x) * patrol.axis_x + (
            point.y - patrol.center.y
        ) * patrol.axis_y

    def option(self, along_m: float) -> _CrossingOption | None:
        base = self._patrol.point_along(along_m)
        staging = Point2D(
            base.x + self._toward_x * self._offset_m,
            base.y + self._toward_y * self._offset_m,
        )
        exit_point = Point2D(
            base.x - self._toward_x * self._offset_m,
            base.y - self._toward_y * self._offset_m,
        )
        arrival_s = distance(self._start, staging) / self._own_speed_mps
        first_step = math.ceil(arrival_s / _CROSSING_STEP_S - 1e-9)
        clear_steps = 0
        for step in range(first_step, len(self._along_m)):
            if abs(self._along_m[step] - along_m) < self._radius_m:
                clear_steps = 0
                continue
            clear_steps += 1
            if clear_steps > self._window_steps:
                start_s = (step - self._window_steps) * _CROSSING_STEP_S
                cost_s = (
                    start_s
                    + self._crossing_s
                    + distance(exit_point, self._goal) / self._own_speed_mps
                )
                return _CrossingOption(cost_s, along_m, staging, exit_point)
        return None

    def options(self) -> list[_CrossingOption]:
        """Feasible crossings, quickest to the goal first."""

        patrol = self._patrol
        reach_m = self._offset_m + _CROSSING_END_REACH_M
        first_m = patrol.lower_m - reach_m
        count = math.floor((patrol.upper_m + reach_m - first_m) / _CROSSING_STEP_M)
        found = (
            self.option(first_m + index * _CROSSING_STEP_M)
            for index in range(count + 1)
        )
        return sorted(
            (option for option in found if option is not None),
            key=lambda option: (option.cost_s, option.along_m),
        )


def _holding_is_safe(
    position: Point2D,
    traffic: tuple[_MovingTraffic, ...],
    *,
    horizon_s: float,
) -> bool:
    for aircraft in traffic:
        if aircraft.patrol is not None:
            if not _sampled_motion_is_safe(
                position, 0.0, 0.0, horizon_s, aircraft
            ):
                return False
        elif (
            _closest_approach_m(position, 0.0, 0.0, aircraft, horizon_s)
            < aircraft.radius_m
        ):
            return False
    return True


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
        self._patrol_history = BackAndForthHistory()
        self._crossing: _PatrolCrossing | None = None

    def reset_planner(self) -> None:
        self._planner.reset()

    def reset(self) -> None:
        self.reset_planner()
        self._patrol_history.reset()
        self._crossing = None

    def _plan_without_patrol(self, request: AvoidanceRequest) -> AvoidancePlan:
        self._planner.reset()
        fallback = self.plan(replace(request, traffic_sample_time_s=None))
        if fallback.status is PlanStatus.PATH_FOUND and fallback.reason is None:
            return replace(fallback, reason="PATROL_FALLBACK")
        return fallback

    def plan(self, request: AvoidanceRequest) -> AvoidancePlan:
        settings = self.config
        position, goal = request.position, request.goal
        if not (_valid_coordinate(position) and _valid_coordinate(goal)):
            self.reset_planner()
            return AvoidancePlan.hold("INVALID_POSITION")
        age_s = request.traffic_age_s
        if not 0.0 <= age_s <= settings.planner.map_freshness_s:
            self.reset_planner()
            return AvoidancePlan.hold("STALE_TRAFFIC")
        aircraft_indices = [aircraft.aircraft_index for aircraft in request.traffic]
        if len(aircraft_indices) != len(set(aircraft_indices)):
            self.reset_planner()
            return AvoidancePlan.hold("DUPLICATE_TRAFFIC_IDENTITY")

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

        patrol_predictions: dict[int, BackAndForthPrediction] = {}
        try:
            patrol_predictions = self._patrol_history.observe(
                request.traffic,
                sample_time_s=request.traffic_sample_time_s,
                origin_lat=position.lat,
                origin_lon=position.lon,
            )
            ordinary_traffic = tuple(
                aircraft
                for aircraft in request.traffic
                if aircraft.aircraft_index not in patrol_predictions
            )
            obstacles = list(
                traffic_to_obstacles(
                    ordinary_traffic,
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
            )
        except ValueError as error:
            self.reset_planner()
            if patrol_predictions:
                return self._plan_without_patrol(request)
            return AvoidancePlan.hold(str(error), goal_distance_m=goal_distance_m)

        moving_traffic = _moving_traffic(
            request.traffic,
            origin_lat=position.lat,
            origin_lon=position.lon,
            age_s=age_s,
            own_speed_mps=settings.horizontal_speed_mps,
            clearance_margin_m=settings.planner.clearance_margin_m,
            patrol_predictions=patrol_predictions,
            patrol_buffer_m=settings.patrol_traffic_buffer_m,
            patrol_model_uncertainty=settings.patrol_model_uncertainty,
        )
        tracks = _traffic_tracks(
            request.traffic,
            moving_traffic,
            self._patrol_history.recent_tracks(
                window_s=settings.traffic_track_window_s,
                origin_lat=position.lat,
                origin_lon=position.lon,
            ),
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
            ) and all(
                _track_crossing_is_safe(
                    start,
                    end,
                    track,
                    own_speed_mps=settings.horizontal_speed_mps,
                    horizon_s=settings.crossing_yield_horizon_s,
                    flight_boundary_is_safe=flight_boundary_is_safe,
                )
                for track in tracks
            )
            dynamic_conflict_seen = dynamic_conflict_seen or not safe
            return safe

        def steer_to(waypoint: Point2D, reason: str) -> AvoidancePlan:
            step_m = math.hypot(waypoint.x, waypoint.y)
            speed_mps = min(settings.horizontal_speed_mps, step_m)
            east_mps = north_mps = 0.0
            if step_m > 1e-6:
                east_mps = speed_mps * waypoint.x / step_m
                north_mps = speed_mps * waypoint.y / step_m
            return AvoidancePlan(
                status=PlanStatus.PATH_FOUND,
                reason=reason,
                east_mps=east_mps,
                north_mps=north_mps,
                up_mps=max(
                    -settings.vertical_speed_mps,
                    min(settings.vertical_speed_mps, altitude_error_m),
                ),
                next_waypoint=enu_to_coordinate(
                    position, waypoint.x, waypoint.y, goal.alt
                ),
                goal_distance_m=goal_distance_m,
                goal_reached=False,
                minimum_clearance_m=None,
                obstacle_count=len(obstacles),
            )

        def retreat_from_track() -> AvoidancePlan | None:
            waypoint = _track_retreat_waypoint(
                start,
                Point2D(goal_east_m, goal_north_m),
                tracks,
                moving_traffic,
                own_speed_mps=settings.horizontal_speed_mps,
                horizon_s=settings.crossing_yield_horizon_s,
                flight_boundary_is_safe=flight_boundary_is_safe,
            )
            if waypoint is None:
                return None
            self._planner.reset()
            return steer_to(waypoint, "RETREATING_FROM_TRAFFIC_TRACK")

        def leg_is_safe(leg_start: Point2D, leg_end: Point2D) -> bool:
            return (
                flight_boundary_is_safe is None
                or flight_boundary_is_safe(leg_start, leg_end)
            ) and dynamic_segment_is_safe(leg_start, leg_end)

        def local(point: Coordinate) -> Point2D:
            return Point2D(
                *east_north_coordinate_offset_m(
                    position.lat, position.lon, point.lat, point.lon
                )
            )

        crossing_subgoal: Point2D | None = None

        def cross_patrol() -> AvoidancePlan | None:
            """
            Carry out, or choose, a crossing of a confirmed patrol line that
            blocks the way to the goal: get to a staging point outside its
            circle, wait there for the predicted gap, then cross in one go.

            Getting to the staging point is left to the normal planner, with
            the staging point as its goal (``crossing_subgoal``), so every
            other aircraft is still avoided on the way.
            """

            nonlocal crossing_subgoal
            now_s = request.traffic_sample_time_s
            if now_s is None:
                # The patrol-free fallback: leave any chosen crossing as is.
                return None
            # Decide against the mission waypoint, not a lookahead point near
            # the drone: whether the line is in the way, and which crossing
            # gets there soonest.
            goal_point = (
                local(request.waypoint)
                if request.waypoint is not None
                else Point2D(goal_east_m, goal_north_m)
            )
            patrols = {
                track.aircraft_index: track
                for track in tracks
                if track.traffic.patrol is not None
            }
            crossing = self._crossing
            if crossing is not None and (
                crossing.aircraft_index not in patrols
                or now_s - crossing.chosen_at_s > _CROSSING_TIMEOUT_S
            ):
                crossing = self._crossing = None

            if crossing is not None and crossing.crossing:
                exit_point = local(crossing.exit)
                staging = local(crossing.staging)
                leg_m = distance(staging, exit_point)
                done_m = (
                    (start.x - staging.x) * (exit_point.x - staging.x)
                    + (start.y - staging.y) * (exit_point.y - staging.y)
                ) / max(leg_m, 1e-9)
                if (
                    distance(start, exit_point) <= _CROSSING_ARRIVAL_M
                    or done_m >= leg_m - _CROSSING_ARRIVAL_M
                ):
                    self._crossing = None
                    return None
                if leg_is_safe(start, exit_point):
                    return steer_to(exit_point, "CROSSING_PATROL")
                self._crossing = None
                return None

            def blocking(track: _TrafficTrack) -> bool:
                # A waypoint inside the band cannot be reached by crossing it.
                band = _patrol_band(track.traffic.patrol, track.traffic.radius_m)
                return not band.contains(
                    Point(start.x, start.y)
                ) and not band.contains(
                    Point(goal_point.x, goal_point.y)
                ) and band.intersects(
                    LineString(((start.x, start.y), (goal_point.x, goal_point.y)))
                )

            def at_crossing(
                staging: Point2D, exit_point: Point2D
            ) -> AvoidancePlan | None:
                nonlocal crossing_subgoal
                if distance(start, staging) > _CROSSING_ARRIVAL_M:
                    # Let the planner get there around everything else.
                    crossing_subgoal = staging
                    return None
                if leg_is_safe(start, exit_point):
                    self._crossing.crossing = True
                    return steer_to(exit_point, "CROSSING_PATROL")
                if _holding_is_safe(
                    start,
                    moving_traffic,
                    horizon_s=settings.crossing_yield_horizon_s,
                ):
                    return steer_to(staging, "WAITING_FOR_PATROL_GAP")
                # Something else is coming for the staging point: plan as
                # usual this cycle, keeping the crossing for later.
                return None

            def search(track: _TrafficTrack) -> _PatrolCrossingSearch:
                return _PatrolCrossingSearch(
                    start,
                    goal_point,
                    track.traffic.patrol,
                    track.traffic.radius_m,
                    track.traffic.patrol_age_s,
                    settings.horizontal_speed_mps,
                )

            if crossing is not None:
                track = patrols[crossing.aircraft_index]
                staging = local(crossing.staging)
                searcher = search(track)
                still_feasible = (
                    blocking(track)
                    and searcher.option(searcher.along_of(staging)) is not None
                )
                if not still_feasible:
                    crossing = self._crossing = None
                else:
                    return at_crossing(staging, local(crossing.exit))

            blockers = sorted(
                (track for track in patrols.values() if blocking(track)),
                key=lambda track: track.line.distance(Point(start.x, start.y)),
            )
            if not blockers:
                return None
            # A confirmed patrol in the way overrides the normal planner. If
            # crossing straight ahead is open, that is the cheapest option.
            track = blockers[0]
            for option in search(track).options():
                if flight_boundary_is_safe is None or flight_boundary_is_safe(
                    option.staging, option.exit
                ):
                    self._crossing = _PatrolCrossing(
                        aircraft_index=track.aircraft_index,
                        staging=enu_to_coordinate(
                            position, option.staging.x, option.staging.y, goal.alt
                        ),
                        exit=enu_to_coordinate(
                            position, option.exit.x, option.exit.y, goal.alt
                        ),
                        chosen_at_s=now_s,
                    )
                    self._planner.reset()
                    return at_crossing(option.staging, option.exit)
            return None

        field = CircularObstacleField(obstacles)
        start = Point2D(0.0, 0.0)
        crossing_plan = cross_patrol()
        if crossing_plan is not None:
            return crossing_plan
        # Where the normal planner heads this cycle: a patrol crossing's
        # staging point while approaching one, otherwise the goal.
        target = (
            crossing_subgoal
            if crossing_subgoal is not None
            else Point2D(goal_east_m, goal_north_m)
        )
        current_conflicts = tuple(
            aircraft
            for aircraft in moving_traffic
            if distance(start, aircraft.position) < aircraft.radius_m
        )
        if current_conflicts:
            retreat = retreat_from_track()
            if retreat is not None:
                return retreat
        if current_conflicts and any(
            aircraft.patrol is not None for aircraft in current_conflicts
        ):
            return self._plan_without_patrol(request)
        if current_conflicts:
            egress = _egress_waypoint(
                start,
                target,
                moving_traffic,
                candidate_headings=self._planner.candidate_headings(
                    heading(start, target)
                ),
                lookahead_m=settings.planner.first_lookahead_m,
                own_speed_mps=settings.horizontal_speed_mps,
                flight_boundary_is_safe=flight_boundary_is_safe,
            )
            self._planner.reset()
            if egress is None:
                if patrol_predictions:
                    return self._plan_without_patrol(request)
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
                goal=target,
                obstacles=ObstacleSnapshot(obstacles, timestamp_s=-age_s),
                now_s=0.0,
            ),
            dynamic_segment_is_safe=dynamic_segment_is_safe,
            flight_boundary_is_safe=flight_boundary_is_safe,
        )
        if result.status is PlanStatus.NO_PATH or result.waypoint is None:
            retreat = retreat_from_track()
            if retreat is not None:
                return retreat
            if patrol_predictions:
                return self._plan_without_patrol(request)
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

        # Reaching a staging point is not reaching the goal.
        reached = (
            crossing_subgoal is None
            and goal_distance_m <= settings.goal_tolerance_m
        )
        if reached and not _holding_is_safe(
            start,
            moving_traffic,
            horizon_s=settings.crossing_yield_horizon_s,
        ):
            self.reset_planner()
            if patrol_predictions:
                return self._plan_without_patrol(request)
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
            reason=(
                "APPROACHING_PATROL_CROSSING"
                if crossing_subgoal is not None
                else "PATROL_PATTERN_ACTIVE"
                if patrol_predictions
                else None
            ),
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