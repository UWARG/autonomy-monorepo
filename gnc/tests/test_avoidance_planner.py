import math
import random

from obstacle_avoidance import (
    BendyRuler2D,
    CircleObstacle,
    CircularObstacleField,
    NoPathReason,
    ObstacleSnapshot,
    PlannerConfig,
    PlanRequest,
    PlanStatus,
    Point2D,
)


def request(
    obstacles: tuple[CircleObstacle, ...] = (),
    *,
    timestamp_s: float = 10.0,
    now_s: float = 10.0,
    healthy: bool = True,
) -> PlanRequest:
    return PlanRequest(
        start=Point2D(0.0, 0.0),
        goal=Point2D(20.0, 0.0),
        obstacles=ObstacleSnapshot(obstacles, timestamp_s, healthy),
        now_s=now_s,
    )


def test_clear_space_selects_direct_16m_first_leg() -> None:
    result = BendyRuler2D().plan(request())
    assert result.status is PlanStatus.PATH_FOUND
    assert result.waypoint == Point2D(16.0, 0.0)
    assert result.second_waypoint == result.waypoint
    assert result.heading_rad == 0.0


def test_default_candidates_cover_full_circle_without_duplicate_opposite_heading() -> None:
    headings = BendyRuler2D().candidate_headings(goal_heading=0.0)

    assert len(headings) == 24
    assert any(math.isclose(abs(value), math.pi) for value in headings)
    assert len({round(value, 12) for value in headings}) == len(headings)


def test_stale_and_future_maps_fail_closed() -> None:
    planner = BendyRuler2D(PlannerConfig(map_freshness_s=0.5))
    stale = planner.plan(request(timestamp_s=9.0, now_s=10.0))
    future = planner.plan(request(timestamp_s=10.1, now_s=10.0))
    assert stale.status is PlanStatus.NO_PATH
    assert stale.reason is NoPathReason.STALE_MAP
    assert future.reason is NoPathReason.STALE_MAP
    assert stale.waypoint is None


def test_unhealthy_map_fails_closed() -> None:
    result = BendyRuler2D().plan(request(healthy=False))
    assert result.status is PlanStatus.NO_PATH
    assert result.reason is NoPathReason.UNHEALTHY_MAP


def test_fully_blocked_start_returns_no_path() -> None:
    result = BendyRuler2D().plan(request((CircleObstacle(Point2D(0.0, 0.0), 2.0),)))
    assert result.status is PlanStatus.NO_PATH
    assert result.reason is NoPathReason.BLOCKED


def test_obstacle_beyond_16m_does_not_create_a_second_leg() -> None:
    obstacle = CircleObstacle(Point2D(20.0, 0.0), 1.0)
    plan_request = request((obstacle,))

    result = BendyRuler2D().plan(plan_request)

    assert result.status is PlanStatus.PATH_FOUND
    assert result.waypoint == Point2D(16.0, 0.0)
    assert result.second_waypoint == result.waypoint
    field = CircularObstacleField((obstacle,))
    assert field.segment_clearance(plan_request.start, result.waypoint) >= 1.0


def test_near_goal_does_not_report_success_inside_keep_away() -> None:
    plan_request = PlanRequest(
        start=Point2D(0.0, 0.0),
        goal=Point2D(0.1, 0.0),
        obstacles=ObstacleSnapshot((CircleObstacle(Point2D(0.1, 0.0), 1.0),), 10.0),
        now_s=10.0,
    )
    result = BendyRuler2D().plan(plan_request)

    assert result.status is PlanStatus.NO_PATH
    assert result.reason is NoPathReason.BLOCKED


def test_symmetric_detour_is_deterministically_counter_clockwise() -> None:
    obstacle = CircleObstacle(Point2D(5.0, 0.0), 1.5)
    headings = [
        BendyRuler2D().plan(request((obstacle,))).heading_rad for _ in range(10)
    ]
    assert all(value is not None and value > 0.0 for value in headings)
    assert len(set(headings)) == 1


def test_hysteresis_retains_a_safe_previous_detour() -> None:
    planner = BendyRuler2D(
        PlannerConfig(first_lookahead_m=8.0, hysteresis_cost_m=2.0)
    )
    first = planner.plan(request((CircleObstacle(Point2D(5.0, 0.0), 1.5),)))
    second = planner.plan(request((CircleObstacle(Point2D(5.0, 0.1), 1.5),)))
    assert first.status is PlanStatus.PATH_FOUND
    assert second.status is PlanStatus.PATH_FOUND
    assert first.heading_rad is not None
    assert second.heading_rad is not None
    assert math.copysign(1.0, second.heading_rad) == math.copysign(
        1.0, first.heading_rad
    )


def test_hysteresis_does_not_keep_a_dynamically_unsafe_heading() -> None:
    planner = BendyRuler2D()
    direct = planner.plan(request())

    detour = planner.plan(
        request(),
        dynamic_segment_is_safe=lambda start, end: not math.isclose(
            end.y - start.y, 0.0, abs_tol=1e-12
        ),
    )

    assert direct.heading_rad == 0.0
    assert detour.status is PlanStatus.PATH_FOUND
    assert detour.heading_rad is not None and detour.heading_rad != 0.0


def test_property_every_returned_first_segment_meets_clearance() -> None:
    """Seeded generative check over varied obstacle fields."""
    rng = random.Random(96)
    config = PlannerConfig(clearance_margin_m=0.8)
    for _ in range(250):
        obstacles = tuple(
            CircleObstacle(
                Point2D(rng.uniform(1.0, 18.0), rng.uniform(-8.0, 8.0)),
                rng.uniform(0.1, 1.5),
            )
            for _ in range(rng.randrange(0, 12))
        )
        plan_request = request(obstacles)
        result = BendyRuler2D(config).plan(plan_request)
        if result.status is PlanStatus.NO_PATH:
            continue
        assert result.waypoint is not None
        assert result.second_waypoint is not None
        field = CircularObstacleField(obstacles)
        assert (
            field.segment_clearance(plan_request.start, result.waypoint) + 1e-9
            >= config.clearance_margin_m
        )


def test_clear_direct_line_overrides_a_stale_detour_heading() -> None:
    """Near the goal every heading ties within the hysteresis band; keeping the
    old detour heading there made the drone circle the waypoint."""
    planner = BendyRuler2D()
    detour = planner.plan(request((CircleObstacle(Point2D(5.0, 0.0), 1.5),)))
    assert detour.heading_rad is not None and detour.heading_rad != 0.0

    near_goal = planner.plan(
        PlanRequest(
            start=Point2D(15.0, 0.0),
            goal=Point2D(20.0, 0.0),
            obstacles=ObstacleSnapshot((), 10.0),
            now_s=10.0,
        )
    )
    assert near_goal.heading_rad == 0.0
    assert near_goal.waypoint == Point2D(20.0, 0.0)
