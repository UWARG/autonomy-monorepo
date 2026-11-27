"""Bands around recent traffic tracks: cross them whole or not at all."""

import math
from dataclasses import replace

import pytest
from obstacle_avoidance import ObstacleAvoider
from obstacle_avoidance.avoidance import (
    _MovingTraffic,
    _track_crossing_is_safe,
    _track_retreat_waypoint,
    _TrafficTrack,
)
from obstacle_avoidance.models import AvoidanceRequest, Point2D, TrafficAircraft
from obstacle_avoidance.patrol import (
    BACK_AND_FORTH,
    NOT_BACK_AND_FORTH,
    UNKNOWN,
    classify_path,
)
from shapely.geometry import LineString, Point

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

HERE = Coordinate(43.0, -80.0, 10.0)
RADIUS_M = 24.0


def _track(aircraft_x: float, east_mps: float = 15.0) -> _TrafficTrack:
    """An aircraft that has flown along y = 0 from x = -40 to 40."""

    line = LineString([(x, 0.0) for x in range(-40, 41, 10)])
    motion = _MovingTraffic(
        position=Point2D(aircraft_x, 0.0),
        east_mps=east_mps,
        north_mps=0.0,
        radius_m=RADIUS_M,
    )
    return _TrafficTrack(
        line=line, band=line.buffer(RADIUS_M), traffic=motion, speed_mps=15.0
    )


def _crossing_is_safe(track: _TrafficTrack, end: Point2D, own_speed_mps: float) -> bool:
    return _track_crossing_is_safe(
        Point2D(0.0, -30.0),
        end,
        track,
        own_speed_mps=own_speed_mps,
        horizon_s=20.0,
        flight_boundary_is_safe=None,
    )


def test_leg_ending_inside_the_band_is_rejected_while_the_aircraft_could_return() -> None:
    # Heading away at the far end of its track: a straight-line prediction
    # calls this clear, but it can turn back before a 2 m/s drone gets across.
    assert not _crossing_is_safe(_track(40.0), Point2D(0.0, -14.0), 2.0)


def test_leg_ending_inside_the_band_is_allowed_when_the_whole_crossing_is_clear() -> None:
    # Far down the track: it cannot get back before the drone is across.
    assert _crossing_is_safe(_track(400.0), Point2D(0.0, -14.0), 10.0)


def test_leg_short_of_the_band_is_unaffected() -> None:
    assert _crossing_is_safe(_track(0.0), Point2D(0.0, -26.0), 2.0)


def test_crossing_that_cannot_leave_the_band_is_rejected() -> None:
    # Flying along the band, it never comes out the other side within reach.
    track = _track(400.0)
    assert not _track_crossing_is_safe(
        Point2D(-60.0, -5.0),
        Point2D(-44.0, -5.0),
        track,
        own_speed_mps=2.0,
        horizon_s=20.0,
        flight_boundary_is_safe=None,
    )


def _retreat(start: Point2D, goal: Point2D) -> Point2D | None:
    track = _track(40.0)
    return _track_retreat_waypoint(
        start,
        goal,
        (track,),
        (track.traffic,),
        own_speed_mps=2.0,
        horizon_s=20.0,
        flight_boundary_is_safe=None,
    )


def test_inside_the_band_retreats_straight_away_from_the_track() -> None:
    waypoint = _retreat(Point2D(0.0, -5.0), Point2D(0.0, 60.0))

    assert waypoint is not None
    assert waypoint.x == pytest.approx(0.0)
    assert waypoint.y == pytest.approx(-(RADIUS_M + 1.0))


def test_on_the_track_retreats_to_the_goal_side() -> None:
    waypoint = _retreat(Point2D(0.0, 0.0), Point2D(0.0, 60.0))

    assert waypoint is not None
    assert waypoint.y == pytest.approx(RADIUS_M + 1.0)


def test_outside_every_band_there_is_nothing_to_retreat_from() -> None:
    assert _retreat(Point2D(0.0, -30.0), Point2D(0.0, 60.0)) is None


def _aircraft(east_m: float, heading_deg: float) -> TrafficAircraft:
    position = enu_to_coordinate(HERE, east_m, 0.0, HERE.alt)
    return TrafficAircraft(
        aircraft_index=1,
        lat=position.lat,
        lon=position.lon,
        altitude_agl_m=10.0,
        speed_mps=15.0,
        heading_deg=heading_deg,
        horizontal_keep_away_m=5.0,
        vertical_keep_away_m=5.0,
    )


def test_avoider_does_not_step_into_a_band_the_aircraft_just_left() -> None:
    """The trap: once it has reversed, never stop where it can come back."""

    avoider = ObstacleAvoider()
    drone = enu_to_coordinate(HERE, 0.0, -30.0, HERE.alt)
    goal = enu_to_coordinate(HERE, 0.0, 60.0, HERE.alt)
    plan = None
    # One pass east along y = 0, then its first turn: a reversal, back west.
    passes = [(float(east_m), 90.0) for east_m in range(-40, 41, 15)]
    passes += [(25.0, 270.0), (10.0, 270.0)]
    for second, (east_m, heading_deg) in enumerate(passes):
        plan = avoider.plan(
            AvoidanceRequest(
                position=drone,
                goal=goal,
                traffic=(_aircraft(east_m, heading_deg),),
                traffic_age_s=0.0,
                traffic_sample_time_s=float(second),
            )
        )

    assert avoider._patrol_history.classification(1) == "BACK_AND_FORTH"
    assert plan is not None
    if plan.next_waypoint is not None and not plan.goal_reached:
        east_m, north_m = east_north_coordinate_offset_m(
            HERE.lat, HERE.lon, plan.next_waypoint.lat, plan.next_waypoint.lon
        )
        track = LineString([(x, 0.0) for x in range(-40, 41, 15)])
        # Radius: keep-away 5 + live buffer 18 at 15 m/s + margin 1.
        assert track.distance(Point(east_m, north_m)) >= 24.0 - 1e-6
        assert not math.isclose(north_m, 60.0)


def _loop(corners: list[tuple[float, float]], speed_mps: float, seconds: int):
    """1 Hz positions flying the closed polygon ``corners``."""

    legs = list(zip(corners, corners[1:] + corners[:1]))
    lengths = [math.dist(a, b) for a, b in legs]
    points = []
    for second in range(seconds):
        travelled_m = (speed_mps * second) % sum(lengths)
        for (a, b), length_m in zip(legs, lengths):
            if travelled_m <= length_m:
                f = travelled_m / length_m
                points.append(Point2D(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
                break
            travelled_m -= length_m
    return tuple(points)


@pytest.mark.parametrize("speed_mps", [5.0, 10.0, 15.0])
def test_a_reversal_as_first_turn_is_back_and_forth(speed_mps: float) -> None:
    points = _loop([(-40.0, 0.0), (40.0, 0.0)], speed_mps, 40)

    assert classify_path(points) == BACK_AND_FORTH


@pytest.mark.parametrize("speed_mps", [5.0, 10.0, 15.0])
@pytest.mark.parametrize(
    "corners",
    [
        [(30.0, 40.0), (60.0, -30.0), (90.0, 40.0)],  # triangle
        [(0.0, 0.0), (60.0, 0.0), (60.0, 60.0), (0.0, 60.0)],  # square
        [(0.0, 0.0), (80.0, 0.0), (40.0, 30.0)],  # flat triangle
    ],
)
def test_a_corner_as_first_turn_is_not_back_and_forth(
    corners: list[tuple[float, float]], speed_mps: float
) -> None:
    points = _loop(corners, speed_mps, 60)

    assert classify_path(points) == NOT_BACK_AND_FORTH


def test_straight_and_stationary_traffic_has_not_turned_yet() -> None:
    straight = tuple(Point2D(10.0 * second, 0.0) for second in range(20))
    hovering = tuple(
        Point2D(0.3 * math.sin(second), 0.3 * math.cos(second)) for second in range(20)
    )

    assert classify_path(straight) == UNKNOWN
    assert classify_path(hovering) == UNKNOWN


def test_polygon_traffic_gets_no_band_so_the_drone_is_not_held() -> None:
    """A triangle flyer is handled as ordinary moving traffic, never as a patrol."""

    avoider = ObstacleAvoider()
    drone = enu_to_coordinate(HERE, 0.0, -30.0, HERE.alt)
    goal = enu_to_coordinate(HERE, 0.0, 60.0, HERE.alt)
    for second, point in enumerate(_loop([(-30.0, 25.0), (30.0, 25.0), (0.0, 55.0)], 10.0, 60)):
        position = enu_to_coordinate(HERE, point.x, point.y, HERE.alt)
        avoider.plan(
            AvoidanceRequest(
                position=drone,
                goal=goal,
                traffic=(replace(_aircraft(0.0, 90.0), lat=position.lat, lon=position.lon, speed_mps=10.0),),
                traffic_age_s=0.0,
                traffic_sample_time_s=float(second),
            )
        )

    history = avoider._patrol_history
    assert history.classification(1) == NOT_BACK_AND_FORTH
    assert history.recent_tracks(window_s=30.0, origin_lat=HERE.lat, origin_lon=HERE.lon) == {}
