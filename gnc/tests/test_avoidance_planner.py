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


def test_clear_space_selects_direct_two_segment_path() -> None:
    result = BendyRuler2D().plan(request())
    assert result.status is PlanStatus.PATH_FOUND
    assert result.waypoint == Point2D(8.0, 0.0)
    assert result.second_waypoint == Point2D(16.0, 0.0)
    assert result.heading_rad == 0.0


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


def test_symmetric_detour_is_deterministically_counter_clockwise() -> None:
    obstacle = CircleObstacle(Point2D(5.0, 0.0), 1.5)
    headings = [
        BendyRuler2D().plan(request((obstacle,))).heading_rad for _ in range(10)
    ]
    assert all(value is not None and value > 0.0 for value in headings)
    assert len(set(headings)) == 1


def test_hysteresis_retains_a_safe_previous_detour() -> None:
    planner = BendyRuler2D(PlannerConfig(hysteresis_cost_m=2.0))
    first = planner.plan(request((CircleObstacle(Point2D(5.0, 0.0), 1.5),)))
    second = planner.plan(request((CircleObstacle(Point2D(5.0, 0.1), 1.5),)))
    assert first.status is PlanStatus.PATH_FOUND
    assert second.status is PlanStatus.PATH_FOUND
    assert first.heading_rad is not None
    assert second.heading_rad is not None
    assert math.copysign(1.0, second.heading_rad) == math.copysign(
        1.0, first.heading_rad
    )


def test_property_every_returned_segment_meets_clearance() -> None:
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
        assert (
            field.segment_clearance(result.waypoint, result.second_waypoint) + 1e-9
            >= config.clearance_margin_m
        )
