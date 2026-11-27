"""Crossing a confirmed back-and-forth line at a slow drone speed."""

import math

import pytest
from obstacle_avoidance import ObstacleAvoider, TrafficAircraft
from obstacle_avoidance.avoidance import AvoidanceConfig, _PatrolCrossingSearch
from obstacle_avoidance.models import AvoidanceRequest, PlanStatus, Point2D
from obstacle_avoidance.patrol import BackAndForthPrediction

from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

HERE = Coordinate(43.0, -80.0, 10.0)
SPAN_M = 80.0
TRAFFIC_MPS = 15.0
KEEP_AWAY_M = 5.0
DRONE_MPS = 2.0


def _patrol() -> BackAndForthPrediction:
    """15 m/s along y = 0 between x = -40 and 40, at the middle heading east."""

    return BackAndForthPrediction(
        center=Point2D(0.0, 0.0),
        axis_x=1.0,
        axis_y=0.0,
        lower_m=-SPAN_M / 2,
        upper_m=SPAN_M / 2,
        current_m=0.0,
        direction=1,
        speed_mps=TRAFFIC_MPS,
        period_s=2 * SPAN_M / TRAFFIC_MPS,
        uncertainty_m=0.0,
        uncertainty_growth_mps=0.0,
    )


def _search(start: Point2D) -> _PatrolCrossingSearch:
    return _PatrolCrossingSearch(
        start, Point2D(0.0, 60.0), _patrol(), 6.0, 0.0, DRONE_MPS
    )


def test_middle_of_a_fast_line_has_no_gap_for_a_slow_drone() -> None:
    # Passes every 5.3 s; a 2 m/s crossing of the band takes 8 s.
    assert _search(Point2D(0.0, -20.0)).option(0.0) is None


def test_near_an_end_there_is_a_gap_and_it_is_preferred() -> None:
    search = _search(Point2D(0.0, -20.0))
    options = search.options()

    assert options
    best = options[0]
    assert abs(best.along_m) > 20.0
    assert best.cost_s == min(option.cost_s for option in options)


def test_staging_and_exit_are_outside_the_circle_on_either_side() -> None:
    option = _search(Point2D(0.0, -20.0)).options()[0]

    assert option.staging.y == pytest.approx(-8.0)
    assert option.exit.y == pytest.approx(8.0)
    assert option.staging.x == pytest.approx(option.along_m)


def _aircraft_east_m(time_s: float) -> tuple[float, float]:
    leg_s = SPAN_M / TRAFFIC_MPS
    phase_s = time_s % (2 * leg_s)
    if phase_s < leg_s:
        return -SPAN_M / 2 + TRAFFIC_MPS * phase_s, 90.0
    return SPAN_M / 2 - TRAFFIC_MPS * (phase_s - leg_s), 270.0


def _fly(
    avoider: ObstacleAvoider,
    until_s: float,
    rate_hz: float = 5.0,
    lookahead_m: float | None = None,
):
    """Fly a 2 m/s drone from y = -40 toward y = 60 across the patrol line."""

    drone_x, drone_y = 0.0, -40.0
    goal = enu_to_coordinate(HERE, 0.0, 60.0, HERE.alt)
    snapshot, snapshot_s = None, -math.inf
    reasons, closest_m = [], math.inf
    step_s = 1.0 / rate_hz
    time_s = 0.0
    while time_s < until_s and drone_y < 30.0:
        if time_s - snapshot_s >= 1.0 - 1e-9:
            east_m, heading_deg = _aircraft_east_m(time_s)
            point = enu_to_coordinate(HERE, east_m, 0.0, HERE.alt)
            snapshot = TrafficAircraft(
                1, point.lat, point.lon, 10.0, TRAFFIC_MPS, heading_deg,
                KEEP_AWAY_M, 5.0,
            )
            snapshot_s = time_s
        east_m, _ = _aircraft_east_m(time_s)
        closest_m = min(closest_m, math.hypot(east_m - drone_x, drone_y))
        position = enu_to_coordinate(HERE, drone_x, drone_y, HERE.alt)
        plan = avoider.plan(
            AvoidanceRequest(
                position=position,
                goal=(
                    goal
                    if lookahead_m is None
                    # Like the position controller: a point a fixed distance
                    # ahead along the path, with the waypoint passed alongside.
                    else enu_to_coordinate(
                        HERE, 0.0, min(60.0, drone_y + lookahead_m), HERE.alt
                    )
                ),
                traffic=(snapshot,),
                traffic_age_s=time_s - snapshot_s,
                traffic_sample_time_s=snapshot_s,
                waypoint=None if lookahead_m is None else goal,
            )
        )
        reasons.append(plan.reason)
        if plan.status is PlanStatus.PATH_FOUND and plan.next_waypoint is not None:
            to_east_m, to_north_m = east_north_coordinate_offset_m(
                position.lat, position.lon,
                plan.next_waypoint.lat, plan.next_waypoint.lon,
            )
            gap_m = math.hypot(to_east_m, to_north_m)
            move_m = min(gap_m, DRONE_MPS * step_s)
            if gap_m > 0.0:
                drone_x += to_east_m / gap_m * move_m
                drone_y += to_north_m / gap_m * move_m
        time_s += step_s
    return drone_y, reasons, closest_m


def test_slow_drone_crosses_a_fast_patrol_line_outside_its_keep_away() -> None:
    avoider = ObstacleAvoider(AvoidanceConfig(horizontal_speed_mps=DRONE_MPS))

    final_y, reasons, closest_m = _fly(avoider, until_s=150.0)

    assert final_y >= 30.0
    assert "APPROACHING_PATROL_CROSSING" in reasons
    assert "CROSSING_PATROL" in reasons
    assert closest_m >= KEEP_AWAY_M


def test_the_patrol_free_fallback_does_not_drop_a_chosen_crossing() -> None:
    avoider = ObstacleAvoider(AvoidanceConfig(horizontal_speed_mps=DRONE_MPS))
    _, reasons, _ = _fly(avoider, until_s=40.0)
    assert "APPROACHING_PATROL_CROSSING" in reasons
    chosen = avoider._crossing
    assert chosen is not None

    position = enu_to_coordinate(HERE, 0.0, -20.0, HERE.alt)
    avoider.plan(
        AvoidanceRequest(
            position=position,
            goal=enu_to_coordinate(HERE, 0.0, 60.0, HERE.alt),
            traffic=(),
            traffic_age_s=0.0,
            traffic_sample_time_s=None,
        )
    )

    assert avoider._crossing is chosen


def test_reset_forgets_a_chosen_crossing() -> None:
    avoider = ObstacleAvoider(AvoidanceConfig(horizontal_speed_mps=DRONE_MPS))
    _fly(avoider, until_s=40.0)
    assert avoider._crossing is not None

    avoider.reset()

    assert avoider._crossing is None


def test_crossing_is_planned_against_the_waypoint_not_the_lookahead_goal() -> None:
    """The position controller's goal is 16 m ahead; the line still gets crossed."""

    avoider = ObstacleAvoider(AvoidanceConfig(horizontal_speed_mps=DRONE_MPS))

    final_y, reasons, closest_m = _fly(avoider, until_s=150.0, lookahead_m=16.0)

    assert final_y >= 30.0
    assert "CROSSING_PATROL" in reasons
    assert closest_m >= KEEP_AWAY_M
