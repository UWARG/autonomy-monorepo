from collections import Counter
from dataclasses import replace

import pytest

from orthomosaic import (
    CameraSpec,
    DroneSpec,
    ScanRequest,
    plan_orthomosaic_path,
    plan_swarm_orthomosaic,
)
from orthomosaic.swarm import _partition_passes
from orthomosaic.sweep import CapturePass
from utils.src.coordinates import enu_to_coordinate
from utils.src.types import Coordinate

ANCHOR = Coordinate(43.5, -80.5, 0.0)
CAMERA = CameraSpec(1280, 720, 1.396, 0.960)
RECTANGLE = [(0.0, 0.0), (200.0, 0.0), (200.0, 70.0), (0.0, 70.0)]
CONCAVE = [
    (0.0, 0.0),
    (200.0, 0.0),
    (200.0, 200.0),
    (140.0, 200.0),
    (140.0, 100.0),
    (60.0, 100.0),
    (60.0, 200.0),
    (0.0, 200.0),
]


def scan_request(points: list[tuple[float, float]]) -> ScanRequest:
    boundary = [enu_to_coordinate(ANCHOR, east, north) for east, north in points]
    return ScanRequest(boundary, CAMERA, target_gsd_m=0.03)


@pytest.mark.parametrize("points", [RECTANGLE, CONCAVE])
def test_unknown_starts_split_the_shared_sweep_in_id_order(
    points: list[tuple[float, float]],
) -> None:
    request = scan_request(points)
    expected = plan_orthomosaic_path(request).capture_waypoints

    success, setpoints = plan_swarm_orthomosaic(
        request, [DroneSpec("bravo"), DroneSpec("alpha")]
    )

    assert success
    assert setpoints is not None
    assert setpoints["alpha"] and setpoints["bravo"]
    assert setpoints["alpha"] + setpoints["bravo"] == expected


def test_known_start_gets_nearby_segment_in_reverse() -> None:
    request = scan_request(RECTANGLE)
    success, unassigned = plan_swarm_orthomosaic(
        request, [DroneSpec("alpha"), DroneSpec("bravo")]
    )
    assert success
    assert unassigned is not None

    known_start = unassigned["bravo"][-1]
    success, setpoints = plan_swarm_orthomosaic(
        request, [DroneSpec("alpha", known_start), DroneSpec("bravo")]
    )

    assert success
    assert setpoints == {
        "alpha": unassigned["bravo"][::-1],
        "bravo": unassigned["alpha"],
    }


def test_one_drone_matches_the_existing_planner() -> None:
    request = scan_request(CONCAVE)
    start = request.boundary[-1]
    expected = plan_orthomosaic_path(replace(request, start=start)).capture_waypoints

    assert plan_swarm_orthomosaic(request, [DroneSpec("solo", start)]) == (
        True,
        {"solo": expected},
    )


def test_extra_drones_get_empty_routes() -> None:
    request = scan_request([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0)])
    expected = plan_orthomosaic_path(request).capture_waypoints
    drones = [
        DroneSpec("alpha"),
        DroneSpec("bravo", request.boundary[0]),
        DroneSpec("charlie"),
    ]

    assert plan_swarm_orthomosaic(request, drones) == (
        True,
        {"alpha": [], "bravo": expected, "charlie": []},
    )


def test_assignment_keeps_all_captures_with_multiple_known_starts() -> None:
    request = scan_request(CONCAVE)
    expected = plan_orthomosaic_path(request).capture_waypoints
    drones = [
        DroneSpec("alpha", request.boundary[0]),
        DroneSpec("bravo", request.boundary[2]),
        DroneSpec("charlie", request.boundary[-1]),
    ]

    success, setpoints = plan_swarm_orthomosaic(request, drones)

    assert success
    assert setpoints is not None
    assert all(setpoints[drone.drone_id] for drone in drones)
    assert Counter(point for route in setpoints.values() for point in route) == Counter(
        expected
    )


@pytest.mark.parametrize(
    "drones",
    [
        [],
        [DroneSpec(" ")],
        [DroneSpec("alpha"), DroneSpec("alpha")],
        [DroneSpec("alpha", Coordinate(float("nan"), -80.5, 0.0))],
    ],
)
def test_invalid_drone_specs_fail(drones: list[DroneSpec]) -> None:
    assert plan_swarm_orthomosaic(scan_request(RECTANGLE), drones) == (False, None)


def test_invalid_survey_fails() -> None:
    request = scan_request(RECTANGLE[:2])
    assert plan_swarm_orthomosaic(
        request, [DroneSpec("alpha"), DroneSpec("bravo")]
    ) == (
        False,
        None,
    )


def test_partition_keeps_long_pass_intact() -> None:
    passes = [
        CapturePass([(float(index), 0.0)], length)
        for index, length in enumerate([1.0, 1.0, 1.0, 10.0])
    ]

    assert _partition_passes(passes, 2) == [
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        [(3.0, 0.0)],
    ]
