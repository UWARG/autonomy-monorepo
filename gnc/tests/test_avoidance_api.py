import math

import pytest

from obstacle_avoidance import (
    AvoidanceConfig,
    AvoidanceRequest,
    ObstacleAvoider,
    PlannerConfig,
    PlanStatus,
    TrafficAircraft,
    live_traffic_buffer_m,
)
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate
from utils.src.waypoint_utils import east_north_coordinate_offset_m

HERE = Coordinate(43.0, -80.0, 10.0)


def offset(east_m: float, north_m: float, alt_m: float = 10.0) -> Coordinate:
    return enu_to_coordinate(HERE, east_m, north_m, alt_m)


def request(
    goal: Coordinate, *traffic: TrafficAircraft, age_s: float = 0.0
) -> AvoidanceRequest:
    return AvoidanceRequest(
        position=HERE, goal=goal, traffic=traffic, traffic_age_s=age_s
    )


def intruder(
    center: Coordinate,
    keep_away_m: float,
    *,
    index: int = 1,
    speed_mps: float = 0.0,
    heading_deg: float = 0.0,
) -> TrafficAircraft:
    return TrafficAircraft(
        aircraft_index=index,
        lat=center.lat,
        lon=center.lon,
        altitude_agl_m=10.0,
        speed_mps=speed_mps,
        heading_deg=heading_deg,
        horizontal_keep_away_m=keep_away_m,
        vertical_keep_away_m=5.0,
    )


def ring(
    radius_m: float, keep_away_m: float, count: int = 8
) -> tuple[TrafficAircraft, ...]:
    """Stationary traffic all round, overlapping into a closed ring."""
    return tuple(
        intruder(
            offset(
                radius_m * math.sin(2 * math.pi * i / count),
                radius_m * math.cos(2 * math.pi * i / count),
            ),
            keep_away_m,
            index=i,
        )
        for i in range(count)
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


def test_flight_boundary_rejects_crossing_candidate_before_route_selection() -> None:
    boundary = tuple(
        offset(east_m, north_m)
        for east_m, north_m in (
            (-12.0, -12.0),
            (12.0, -12.0),
            (12.0, 12.0),
            (-12.0, 12.0),
        )
    )
    avoider = ObstacleAvoider(
        AvoidanceConfig(
            flight_boundary=boundary,
            planner=PlannerConfig(first_lookahead_m=16.0),
        )
    )

    plan = avoider.plan(request(offset(0.0, 30.0)))

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.next_waypoint is not None
    east_m, north_m = east_north_coordinate_offset_m(
        HERE.lat, HERE.lon, plan.next_waypoint.lat, plan.next_waypoint.lon
    )
    assert math.hypot(east_m, north_m) == pytest.approx(16.0)
    assert abs(east_m) > 8.0
    assert abs(east_m) <= 12.0
    assert abs(north_m) <= 12.0


def test_flight_boundary_allows_safe_reentry_from_outside() -> None:
    boundary = tuple(
        offset(east_m, north_m)
        for east_m, north_m in (
            (10.0, -10.0),
            (30.0, -10.0),
            (30.0, 10.0),
            (10.0, 10.0),
        )
    )
    avoider = ObstacleAvoider(
        AvoidanceConfig(
            flight_boundary=boundary,
            planner=PlannerConfig(first_lookahead_m=4.0),
        )
    )

    plan = avoider.plan(request(offset(20.0, 0.0)))

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.next_waypoint is not None
    east_m, north_m = east_north_coordinate_offset_m(
        HERE.lat, HERE.lon, plan.next_waypoint.lat, plan.next_waypoint.lon
    )
    assert east_m == pytest.approx(4.0, abs=0.1)
    assert north_m == pytest.approx(0.0, abs=0.1)


def test_flight_boundary_does_not_allow_outside_path_to_move_away() -> None:
    boundary = tuple(
        offset(east_m, north_m)
        for east_m, north_m in (
            (10.0, -10.0),
            (30.0, -10.0),
            (30.0, 10.0),
            (10.0, 10.0),
        )
    )
    avoider = ObstacleAvoider(
        AvoidanceConfig(
            flight_boundary=boundary,
            planner=PlannerConfig(first_lookahead_m=4.0),
        )
    )

    plan = avoider.plan(request(offset(-20.0, 0.0)))

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.next_waypoint is not None
    east_m, north_m = east_north_coordinate_offset_m(
        HERE.lat, HERE.lon, plan.next_waypoint.lat, plan.next_waypoint.lon
    )
    remaining_distance_to_boundary = math.hypot(
        max(0.0, 10.0 - east_m),
        max(0.0, abs(north_m) - 10.0),
    )
    assert remaining_distance_to_boundary < 10.0


def test_vertical_speed_tracks_goal_altitude_within_limit() -> None:
    assert ObstacleAvoider().plan(request(offset(0.0, 20.0, 13.0))).up_mps == 1.0
    assert ObstacleAvoider().plan(request(offset(0.0, 20.0, 7.0))).up_mps == -1.0


def test_detours_around_traffic_using_live_buffer() -> None:
    plan = ObstacleAvoider().plan(
        request(offset(0.0, 30.0), intruder(offset(0.0, 12.0), 2.0))
    )

    assert plan.status is PlanStatus.PATH_FOUND
    assert abs(plan.east_mps) > 0.1
    assert plan.minimum_clearance_m is not None
    assert plan.minimum_clearance_m >= 1.0


@pytest.mark.parametrize(
    ("plan_request", "reason"),
    [
        (
            request(offset(0.0, 20.0), *ring(6.0, 5.5)),
            "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED",
        ),
        (request(offset(0.0, 20.0), age_s=3.0), "STALE_TRAFFIC"),
        (request(offset(0.0, 20.0), age_s=-0.1), "STALE_TRAFFIC"),
        (request(Coordinate(float("nan"), 0.0, 10.0)), "INVALID_POSITION"),
    ],
)
def test_failures_hold_at_zero_velocity(
    plan_request: AvoidanceRequest, reason: str
) -> None:
    plan = ObstacleAvoider().plan(plan_request)

    assert plan.status is PlanStatus.NO_PATH
    assert plan.reason == reason
    assert (plan.east_mps, plan.north_mps, plan.up_mps) == (0.0, 0.0, 0.0)


def test_goal_reached_within_tolerance_stops() -> None:
    plan = ObstacleAvoider().plan(request(offset(0.0, 0.5)))

    assert plan.goal_reached
    assert (plan.east_mps, plan.north_mps, plan.up_mps) == (0.0, 0.0, 0.0)


def test_egresses_from_inside_a_keep_away_when_separation_increases() -> None:
    plan = ObstacleAvoider().plan(request(offset(0.0, 20.0), intruder(HERE, 20.0)))

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.reason == "EGRESSING_CONFLICT"
    assert plan.next_waypoint is not None
    assert plan.next_waypoint.lat > HERE.lat
    assert plan.minimum_clearance_m is not None
    assert plan.minimum_clearance_m < 0.0


def test_egress_does_not_enter_another_aircraft_keep_away() -> None:
    conflict = intruder(HERE, 2.0, index=1)
    ahead = intruder(offset(0.0, 10.0), 2.0, index=2)

    plan = ObstacleAvoider().plan(
        request(offset(0.0, 40.0), conflict, ahead)
    )

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.reason == "EGRESSING_CONFLICT"
    assert plan.next_waypoint is not None
    east_m, north_m = east_north_coordinate_offset_m(
        HERE.lat, HERE.lon, plan.next_waypoint.lat, plan.next_waypoint.lon
    )
    fraction = min(1.0, max(0.0, 10.0 * north_m / (east_m**2 + north_m**2)))
    nearest_east_m = fraction * east_m
    nearest_north_m = fraction * north_m
    clearance_m = math.hypot(nearest_east_m, nearest_north_m - 10.0) - 3.0
    assert clearance_m >= 0.0


def test_egress_checks_other_keep_away_zones_beyond_first_waypoint() -> None:
    conflict = intruder(HERE, 16.0, index=1)
    farther_ahead = intruder(offset(0.0, 25.0), 2.0, index=2)

    plan = ObstacleAvoider().plan(
        request(offset(0.0, 40.0), conflict, farther_ahead)
    )

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.reason == "EGRESSING_CONFLICT"
    assert plan.next_waypoint is not None
    east_m, north_m = east_north_coordinate_offset_m(
        HERE.lat, HERE.lon, plan.next_waypoint.lat, plan.next_waypoint.lon
    )
    heading_east = east_m / math.hypot(east_m, north_m)
    heading_north = north_m / math.hypot(east_m, north_m)
    simulated_end = (heading_east * 20.0, heading_north * 20.0)
    nearest_fraction = min(
        1.0,
        max(0.0, (25.0 * heading_north) / 20.0),
    )
    nearest_east = nearest_fraction * simulated_end[0]
    nearest_north = nearest_fraction * simulated_end[1]
    assert math.hypot(nearest_east, nearest_north - 25.0) >= 6.0


def test_egress_can_exit_a_large_buffer_beyond_crossing_yield_horizon() -> None:
    oversized_conflict = intruder(HERE, 100.0)

    plan = ObstacleAvoider().plan(
        request(offset(0.0, 20.0), oversized_conflict)
    )

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.reason == "EGRESSING_CONFLICT"
    assert plan.next_waypoint is not None


def test_head_on_traffic_with_no_safe_path_requests_pilot_takeover() -> None:
    oncoming = intruder(offset(0.0, 10.0), 5.0, speed_mps=5.0, heading_deg=180.0)
    plan = ObstacleAvoider().plan(request(offset(0.0, 20.0), oncoming))

    assert plan.status is PlanStatus.NO_PATH
    assert plan.reason == "HOLD_CONFLICT_PILOT_TAKEOVER_REQUIRED"


def test_fresh_snapshot_resumes_normal_planning_after_egress() -> None:
    avoider = ObstacleAvoider()
    egress = avoider.plan(request(offset(0.0, 20.0), intruder(HERE, 20.0)))
    recovered = avoider.plan(request(offset(0.0, 20.0)))

    assert egress.status is PlanStatus.PATH_FOUND
    assert egress.reason == "EGRESSING_CONFLICT"
    assert recovered.status is PlanStatus.PATH_FOUND
    assert recovered.reason is None


def test_traffic_expires_after_one_and_a_half_seconds() -> None:
    assert (
        ObstacleAvoider().plan(request(offset(0.0, 20.0), age_s=1.5)).status
        is PlanStatus.PATH_FOUND
    )
    assert (
        ObstacleAvoider().plan(request(offset(0.0, 20.0), age_s=1.501)).reason
        == "STALE_TRAFFIC"
    )


def test_moving_crossing_filters_direct_candidate_and_finds_safe_detour() -> None:
    crossing = intruder(offset(-15.0, 20.0), 5.0, speed_mps=1.2, heading_deg=90.0)
    avoider = ObstacleAvoider(
        AvoidanceConfig(planner=PlannerConfig(first_lookahead_m=16.0))
    )
    plan = avoider.plan(request(offset(0.0, 40.0), crossing))

    assert plan.status is PlanStatus.PATH_FOUND
    assert plan.next_waypoint is not None
    assert abs(plan.east_mps) > 0.1

    waypoint_east, waypoint_north = east_north_coordinate_offset_m(
        HERE.lat,
        HERE.lon,
        plan.next_waypoint.lat,
        plan.next_waypoint.lon,
    )
    leg_time_s = math.hypot(waypoint_east, waypoint_north) / math.hypot(
        plan.east_mps, plan.north_mps
    )
    relative_east_mps = 1.2 - plan.east_mps
    relative_north_mps = -plan.north_mps
    relative_speed_sq = relative_east_mps**2 + relative_north_mps**2
    closest_s = max(
        0.0,
        min(
            leg_time_s,
            -(
                -15.0 * relative_east_mps + 20.0 * relative_north_mps
            )
            / relative_speed_sq,
        ),
    )
    closest_m = math.hypot(
        -15.0 + relative_east_mps * closest_s,
        20.0 + relative_north_mps * closest_s,
    )
    assert closest_m >= (
        5.0 + live_traffic_buffer_m(1.2, 0.0) + 1.0
    )


def test_yields_when_every_candidate_conflicts_but_hold_is_safe() -> None:
    crossing = intruder(offset(-15.0, 20.0), 5.0, speed_mps=1.2, heading_deg=90.0)
    avoider = ObstacleAvoider(
        AvoidanceConfig(
            planner=PlannerConfig(
                first_lookahead_m=16.0,
                maximum_detour_deg=0.0,
            )
        )
    )

    plan = avoider.plan(request(offset(0.0, 40.0), crossing))

    assert plan.status is PlanStatus.NO_PATH
    assert plan.reason == "YIELD_CROSSING"
    assert plan.next_waypoint is None


def test_crossing_traffic_clear_of_route_allows_progress() -> None:
    past_route = intruder(offset(15.0, 20.0), 5.0, speed_mps=1.2, heading_deg=90.0)
    plan = ObstacleAvoider().plan(request(offset(0.0, 40.0), past_route))

    assert plan.status is PlanStatus.PATH_FOUND
