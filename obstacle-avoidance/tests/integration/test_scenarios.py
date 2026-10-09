import math

from obstacle_avoidance import (
    BendyRuler2D,
    CircleObstacle,
    ObstacleSnapshot,
    PlannerConfig,
    PlanRequest,
    PlanStatus,
    Point2D,
)

DEFAULT_GOAL = Point2D(20.0, 0.0)


def circles(
    points: list[tuple[float, float]], radius_m: float
) -> tuple[CircleObstacle, ...]:
    return tuple(CircleObstacle(Point2D(x, y), radius_m) for x, y in points)


def simulate(
    obstacles: tuple[CircleObstacle, ...],
    goal: Point2D = DEFAULT_GOAL,
    steps: int = 80,
) -> tuple[Point2D, list[float], list[Point2D]]:
    planner = BendyRuler2D(
        PlannerConfig(
            first_lookahead_m=6.0,
            second_lookahead_m=6.0,
            clearance_margin_m=0.7,
            hysteresis_cost_m=0.5,
        )
    )
    position = Point2D(0.0, 0.0)
    headings: list[float] = []
    positions = [position]
    for tick in range(steps):
        result = planner.plan(
            PlanRequest(
                start=position,
                goal=goal,
                obstacles=ObstacleSnapshot(obstacles, tick * 0.1),
                now_s=tick * 0.1,
            )
        )
        if result.status is PlanStatus.NO_PATH:
            break
        if result.heading_rad is not None:
            headings.append(result.heading_rad)
        assert result.waypoint is not None
        delta_x = result.waypoint.x - position.x
        delta_y = result.waypoint.y - position.y
        step_m = min(0.5, math.hypot(delta_x, delta_y))
        direction = math.atan2(delta_y, delta_x)
        position = Point2D(
            position.x + math.cos(direction) * step_m,
            position.y + math.sin(direction) * step_m,
        )
        positions.append(position)
        if math.hypot(goal.x - position.x, goal.y - position.y) <= 0.3:
            break
    return position, headings, positions


def test_wall_scenario_routes_around_finite_end() -> None:
    wall = circles([(8.0, float(y)) for y in range(-3, 4)], 0.45)
    final, headings, _ = simulate(wall)
    assert math.hypot(20.0 - final.x, final.y) <= 0.5
    assert any(abs(value) > math.radians(10.0) for value in headings)


def test_narrow_but_valid_gap_remains_traversable() -> None:
    wall = circles(
        [(8.0, float(y)) for y in range(-6, 7) if abs(y) >= 2],
        0.35,
    )
    final, _, _ = simulate(wall)
    assert math.hypot(20.0 - final.x, final.y) <= 0.5


def test_u_shape_reports_no_path_instead_of_entering_blocked_space() -> None:
    u_shape = circles(
        [(5.0, float(y)) for y in range(-4, 5)]
        + [(float(x), -4.0) for x in range(-2, 6)]
        + [(float(x), 4.0) for x in range(-2, 6)],
        0.65,
    )
    planner = BendyRuler2D(
        PlannerConfig(
            first_lookahead_m=6.0,
            second_lookahead_m=6.0,
            clearance_margin_m=0.7,
            maximum_detour_deg=120.0,
        )
    )
    result = planner.plan(
        PlanRequest(
            start=Point2D(1.0, 0.0),
            goal=Point2D(12.0, 0.0),
            obstacles=ObstacleSnapshot(u_shape, 0.0),
            now_s=0.0,
        )
    )
    assert result.status is PlanStatus.NO_PATH


def test_stale_map_scenario_requires_hold() -> None:
    result = BendyRuler2D().plan(
        PlanRequest(
            start=Point2D(0.0, 0.0),
            goal=Point2D(20.0, 0.0),
            obstacles=ObstacleSnapshot((), 0.0),
            now_s=1.0,
        )
    )
    assert result.status is PlanStatus.NO_PATH
    assert result.waypoint is None


def test_symmetric_detour_stays_on_one_side() -> None:
    wall = circles([(8.0, float(y)) for y in range(-2, 3)], 0.45)
    final, _, positions = simulate(wall)
    crossing_positions = [
        position for position in positions if 4.0 <= position.x <= 10.0
    ]
    assert math.hypot(20.0 - final.x, final.y) <= 0.5
    assert crossing_positions
    assert all(position.y >= -1e-9 for position in crossing_positions)
    assert max(position.y for position in crossing_positions) > 2.5
