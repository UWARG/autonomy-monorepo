"""
Unit tests for turning planner output into ordered Missions.
"""

from __future__ import annotations

import math

import pytest
from gnc.src.orthomosaic import DroneSpec, ScanRequest, plan_swarm_orthomosaic
from utils.src.coordinates import anchor_for_boundary, coordinate_to_enu
from utils.src.types import Coordinate

from src.survey_executive.planning import (
    PlanningError,
    _pass_axis_rad,
    _route_yaws_rad,
    plan_routes,
    task_id,
)
from tests.unit.survey_executive.factories import (
    LAT0,
    LON0,
    RECTANGLE,
    make_drone,
    make_request,
)

NORTH, EAST, SOUTH, WEST = 0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0


def planner_setpoints(drones: list[DroneSpec]) -> dict[str, list[Coordinate]]:
    request = make_request()
    ok, setpoints = plan_swarm_orthomosaic(
        ScanRequest(
            boundary=request.boundary,
            camera=request.camera,
            target_gsd_m=request.target_gsd_m,
            forward_overlap=request.forward_overlap,
            side_overlap=request.side_overlap,
        ),
        drones,
    )
    assert ok and setpoints is not None
    return setpoints


def angle_close(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(math.remainder(a - b, 2.0 * math.pi)) <= tol


def test_task_id_format() -> None:
    assert task_id("survey-1", 0, "drone-01", 7) == "survey-1-r0-drone-01-007"
    assert task_id("s", 2, "d", 123) == "s-r2-d-123"


def test_missions_follow_planner_order_for_each_drone() -> None:
    drones = [make_drone("drone-01"), make_drone("drone-02")]
    routes = plan_routes(make_request(), drones, round_index=0)
    expected = planner_setpoints([DroneSpec("drone-01"), DroneSpec("drone-02")])

    assert set(routes) == {"drone-01", "drone-02"}
    for drone_id, missions in routes.items():
        assert [mission.target for mission in missions] == expected[drone_id]
        assert [mission.task_id for mission in missions] == [
            task_id("survey-1", 0, drone_id, index) for index in range(len(missions))
        ]


def test_every_setpoint_appears_once_with_a_unique_id() -> None:
    drones = [make_drone(f"drone-0{n}") for n in range(1, 4)]
    routes = plan_routes(make_request(), drones, round_index=0)
    expected = planner_setpoints([DroneSpec(d.drone_id) for d in drones])

    missions = [mission for route in routes.values() for mission in route]
    ids = [mission.task_id for mission in missions]
    assert len(ids) == len(set(ids))
    assert len(missions) == sum(len(route) for route in expected.values())
    assert sorted((m.target.lat, m.target.lon) for m in missions) == sorted(
        (p.lat, p.lon) for route in expected.values() for p in route
    )


def test_same_input_gives_the_same_routes() -> None:
    drones = [make_drone("drone-01"), make_drone("drone-02")]
    first = plan_routes(make_request(), drones, round_index=0)
    second = plan_routes(make_request(), drones, round_index=0)
    assert first == second


def test_drone_positions_are_passed_as_planner_starts() -> None:
    corner_a = Coordinate(LAT0, LON0, 0.0)
    corner_b = Coordinate(LAT0 + 0.001, LON0 + 0.001, 0.0)
    drones = [
        make_drone("drone-01", position=corner_a),
        make_drone("drone-02", position=corner_b),
    ]
    routes = plan_routes(make_request(), drones, round_index=0)
    expected = planner_setpoints(
        [DroneSpec("drone-01", start=corner_a), DroneSpec("drone-02", start=corner_b)]
    )
    for drone_id, missions in routes.items():
        assert [mission.target for mission in missions] == expected[drone_id]


def test_boundary_override_and_round_index() -> None:
    # The southern half of the survey, as a reshoot round would plan a gap.
    half = [
        RECTANGLE[0],
        RECTANGLE[1],
        Coordinate(LAT0 + 0.0005, LON0 + 0.001, 0.0),
        Coordinate(LAT0 + 0.0005, LON0, 0.0),
    ]
    full = plan_routes(make_request(), [make_drone()], round_index=0)["drone-01"]
    gap = plan_routes(make_request(), [make_drone()], round_index=1, boundary=half)[
        "drone-01"
    ]

    assert 0 < len(gap) < len(full)
    assert all(m.task_id.startswith("survey-1-r1-drone-01-") for m in gap)
    assert all(LAT0 <= m.target.lat <= LAT0 + 0.0005 for m in gap)


def test_all_setpoints_share_the_planned_altitude() -> None:
    missions = plan_routes(make_request(), [make_drone()], round_index=0)["drone-01"]
    altitudes = {mission.target.alt for mission in missions}
    assert len(altitudes) == 1
    assert altitudes.pop() > 0.0


def test_planned_yaws_face_along_the_passes_both_ways() -> None:
    request = make_request()
    missions = plan_routes(request, [make_drone()], round_index=0)["drone-01"]
    anchor = anchor_for_boundary(request.boundary)
    route = [coordinate_to_enu(anchor, m.target) for m in missions]
    axis = _pass_axis_rad([route])
    assert axis is not None

    for mission in missions:
        assert angle_close(mission.yaw, axis) or angle_close(mission.yaw, axis + math.pi)
    # A lawn-mower flies alternate passes in opposite directions.
    assert any(angle_close(m.yaw, axis) for m in missions)
    assert any(angle_close(m.yaw, axis + math.pi) for m in missions)


def test_yaw_in_the_turn_keeps_facing_along_the_pass() -> None:
    # Two passes north then south, 3 photos each, one eastward turn between.
    route = [(0.0, 0.0), (0.0, 10.0), (0.0, 20.0), (5.0, 20.0), (5.0, 10.0), (5.0, 0.0)]
    axis = _pass_axis_rad([route])
    assert axis is not None and angle_close(axis, NORTH)

    yaws = _route_yaws_rad(route, axis)
    expected = [NORTH, NORTH, NORTH, SOUTH, SOUTH, SOUTH]
    assert all(angle_close(y, e) for y, e in zip(yaws, expected))


def test_pass_axis_ignores_direction_and_wraps_near_north() -> None:
    # Passes 1 degree either side of north, flown both ways.
    tilt = math.radians(1.0)
    route = [
        (0.0, 0.0),
        (10.0 * math.sin(tilt), 10.0 * math.cos(tilt)),
        (5.0, 10.0),
        (5.0 + 10.0 * math.sin(tilt), 0.0),
    ]
    axis = _pass_axis_rad([route])
    assert axis is not None
    # Mean of +1 deg and -1 deg lines is north, not 90 deg.
    assert _axis_gap(axis, NORTH) < math.radians(1.5)


def _axis_gap(a: float, b: float) -> float:
    gap = abs(a - b) % math.pi
    return min(gap, math.pi - gap)


def test_east_west_passes_give_east_and_west_yaws() -> None:
    route = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (20.0, 5.0), (10.0, 5.0), (0.0, 5.0)]
    yaws = _route_yaws_rad(route, _pass_axis_rad([route]))
    expected = [EAST, EAST, EAST, WEST, WEST, WEST]
    assert all(angle_close(y, e) for y, e in zip(yaws, expected))


def test_routes_without_direction_get_zero_yaw() -> None:
    assert _pass_axis_rad([[(0.0, 0.0)], []]) is None
    assert _pass_axis_rad([[(1.0, 1.0), (1.0, 1.0)]]) is None
    assert _route_yaws_rad([(0.0, 0.0)], None) == [0.0]


def test_drone_with_no_passes_gets_an_empty_route() -> None:
    # Far more drones than passes on a small area: some get nothing.
    tiny = [
        RECTANGLE[0],
        Coordinate(LAT0, LON0 + 0.0002, 0.0),
        Coordinate(LAT0 + 0.0002, LON0 + 0.0002, 0.0),
        Coordinate(LAT0 + 0.0002, LON0, 0.0),
    ]
    drones = [make_drone(f"drone-{n:02d}") for n in range(1, 13)]
    routes = plan_routes(make_request(boundary=tiny), drones, round_index=0)
    assert set(routes) == {d.drone_id for d in drones}
    assert any(not route for route in routes.values())
    assert any(route for route in routes.values())


def test_planner_rejection_raises_planning_error() -> None:
    with pytest.raises(PlanningError, match="round 0 of survey 'survey-1'"):
        plan_routes(make_request(), [], round_index=0)
    sliver = [RECTANGLE[0], RECTANGLE[1], RECTANGLE[0]]
    with pytest.raises(PlanningError):
        plan_routes(make_request(), [make_drone()], round_index=1, boundary=sliver)


def test_negative_round_index_is_rejected() -> None:
    with pytest.raises(ValueError, match="round_index"):
        plan_routes(make_request(), [make_drone()], round_index=-1)


def test_duplicate_drone_ids_are_rejected_by_name() -> None:
    with pytest.raises(ValueError, match="duplicate drone IDs"):
        plan_routes(make_request(), [make_drone(), make_drone()], round_index=0)
