import pytest

from obstacle_avoidance import (
    AvoidanceRequest,
    ObstacleAvoider,
    PlanStatus,
    TrafficAircraft,
)
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate

HERE = Coordinate(43.0, -80.0, 10.0)


def offset(east_m: float, north_m: float, alt_m: float = 10.0) -> Coordinate:
    return enu_to_coordinate(HERE, east_m, north_m, alt_m)


def request(goal: Coordinate, *traffic: TrafficAircraft, age_s: float = 0.0) -> AvoidanceRequest:
    return AvoidanceRequest(position=HERE, goal=goal, traffic=traffic, traffic_age_s=age_s)


def intruder(center: Coordinate, keep_away_m: float) -> TrafficAircraft:
    return TrafficAircraft(
        aircraft_index=1,
        lat=center.lat,
        lon=center.lon,
        altitude_agl_m=10.0,
        speed_mps=0.0,
        heading_deg=0.0,
        horizontal_keep_away_m=keep_away_m,
        vertical_keep_away_m=5.0,
    )


def test_clear_path_flies_straight_at_capped_speed() -> None:
    north = ObstacleAvoider().plan(request(offset(0.0, 20.0)))
    east = ObstacleAvoider().plan(request(offset(20.0, 0.0)))

    assert north.status is PlanStatus.PATH_FOUND
    assert north.east_mps == pytest.approx(0.0, abs=1e-6)
    assert north.north_mps == pytest.approx(2.0)
    assert east.east_mps == pytest.approx(2.0)
    assert east.north_mps == pytest.approx(0.0, abs=1e-6)
    assert north.next_waypoint is not None and north.next_waypoint.lat > HERE.lat


def test_vertical_speed_tracks_goal_altitude_within_limit() -> None:
    assert ObstacleAvoider().plan(request(offset(0.0, 20.0, 13.0))).up_mps == 1.0
    assert ObstacleAvoider().plan(request(offset(0.0, 20.0, 7.0))).up_mps == -1.0


def test_detours_around_traffic_on_the_direct_line() -> None:
    plan = ObstacleAvoider().plan(request(offset(0.0, 20.0), intruder(offset(0.0, 6.0), 2.0)))

    assert plan.status is PlanStatus.PATH_FOUND
    assert abs(plan.east_mps) > 0.1
    assert plan.minimum_clearance_m is not None and plan.minimum_clearance_m >= 1.0


@pytest.mark.parametrize(
    ("plan_request", "reason"),
    [
        (request(offset(0.0, 20.0), intruder(HERE, 20.0)), "BLOCKED"),
        (request(offset(0.0, 20.0), age_s=3.0), "STALE_TRAFFIC"),
        (request(offset(0.0, 20.0), age_s=-0.1), "STALE_TRAFFIC"),
        (request(Coordinate(float("nan"), 0.0, 10.0)), "INVALID_POSITION"),
    ],
)
def test_failures_hold_at_zero_velocity(plan_request: AvoidanceRequest, reason: str) -> None:
    plan = ObstacleAvoider().plan(plan_request)

    assert plan.status is PlanStatus.NO_PATH
    assert plan.reason == reason
    assert (plan.east_mps, plan.north_mps, plan.up_mps) == (0.0, 0.0, 0.0)


def test_goal_reached_within_tolerance_stops() -> None:
    plan = ObstacleAvoider().plan(request(offset(0.0, 0.5)))

    assert plan.goal_reached
    assert (plan.east_mps, plan.north_mps, plan.up_mps) == (0.0, 0.0, 0.0)
