"""
Unit tests for survey request validation.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest
from utils.src.types import Coordinate

from src.survey_executive import SurveyRequest, validate_request
from tests.unit.survey_executive.factories import (
    CAMERA,
    LAT0,
    LON0,
    RECTANGLE,
    make_drone,
    make_request,
)


def test_valid_request_passes() -> None:
    validate_request(make_request(), [make_drone("drone-01"), make_drone("drone-02")])


def test_defaults_match_design() -> None:
    request = make_request()
    assert request.forward_overlap == 0.75
    assert request.side_overlap == 0.65
    assert request.max_reshoot_rounds == 2
    assert request.min_gap_area_m2 == 1.0
    assert request.max_gsd_ratio == 1.5
    assert request.max_gsd_m == pytest.approx(1.5 * request.target_gsd_m)


def test_request_is_keyword_only_and_frozen() -> None:
    with pytest.raises(TypeError):
        SurveyRequest("survey-1", RECTANGLE, CAMERA, 0.03)  # type: ignore[misc]
    with pytest.raises(AttributeError):
        make_request().target_gsd_m = 0.05  # type: ignore[misc]


def test_drone_with_no_position_yet_is_allowed() -> None:
    # The planner accepts drones without a start; only connection is required.
    validate_request(make_request(), [make_drone(position=None)])


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"survey_id": "  "}, "survey_id"),
        ({"boundary": RECTANGLE[:2]}, "at least 3 points"),
        ({"boundary": []}, "at least 3 points"),
        ({"boundary": [*RECTANGLE[:3], Coordinate(91.0, LON0, 0.0)]}, "valid GPS"),
        ({"boundary": [*RECTANGLE[:3], Coordinate(LAT0, 181.0, 0.0)]}, "valid GPS"),
        # Bow tie: edges 0-1 and 2-3 cross.
        (
            {"boundary": [RECTANGLE[0], RECTANGLE[2], RECTANGLE[1], RECTANGLE[3]]},
            "self-intersecting",
        ),
        ({"boundary": [RECTANGLE[0], RECTANGLE[1], RECTANGLE[0]]}, "boundary polygon"),
        (
            {"camera": replace(CAMERA, image_width_px=0)},
            "image dimensions",
        ),
        (
            {"camera": replace(CAMERA, horizontal_fov_rad=math.pi)},
            "horizontal_fov_rad",
        ),
        (
            {"camera": replace(CAMERA, vertical_fov_rad=0.0)},
            "vertical_fov_rad",
        ),
        ({"target_gsd_m": 0.0}, "target_gsd_m"),
        ({"target_gsd_m": -0.01}, "target_gsd_m"),
        ({"target_gsd_m": math.nan}, "target_gsd_m"),
        ({"target_gsd_m": math.inf}, "target_gsd_m"),
        ({"forward_overlap": 1.0}, "forward_overlap"),
        ({"forward_overlap": -0.1}, "forward_overlap"),
        ({"side_overlap": 1.0}, "side_overlap"),
        ({"side_overlap": -0.1}, "side_overlap"),
        ({"min_gap_area_m2": -1.0}, "min_gap_area_m2"),
        ({"min_gap_area_m2": math.nan}, "min_gap_area_m2"),
        ({"max_gsd_ratio": 0.99}, "max_gsd_ratio"),
        ({"max_gsd_ratio": math.inf}, "max_gsd_ratio"),
        ({"max_gsd_ratio": math.nan}, "max_gsd_ratio"),
        ({"max_reshoot_rounds": -1}, "max_reshoot_rounds"),
        ({"max_reshoot_rounds": 1.5}, "max_reshoot_rounds"),
        ({"max_reshoot_rounds": True}, "max_reshoot_rounds"),
    ],
)
def test_invalid_request_is_rejected(overrides: dict[str, object], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        validate_request(make_request(**overrides), [make_drone()])


def test_overlap_and_gap_edges_are_allowed() -> None:
    request = make_request(
        forward_overlap=0.0,
        side_overlap=0.0,
        min_gap_area_m2=0.0,
        max_gsd_ratio=1.0,
        max_reshoot_rounds=0,
    )
    validate_request(request, [make_drone()])


def test_no_drones_is_rejected() -> None:
    with pytest.raises(ValueError, match="at least one drone"):
        validate_request(make_request(), [])


def test_duplicate_drone_ids_are_rejected() -> None:
    with pytest.raises(ValueError, match="duplicate drone_id 'drone-01'"):
        validate_request(make_request(), [make_drone("drone-01"), make_drone("drone-01")])


def test_blank_drone_id_is_rejected() -> None:
    with pytest.raises(ValueError, match="drone_id must not be blank"):
        validate_request(make_request(), [make_drone(" ")])


def test_disconnected_drone_is_rejected() -> None:
    drones = [make_drone("drone-01"), make_drone("drone-02", connected=False)]
    with pytest.raises(ValueError, match="'drone-02' is not connected"):
        validate_request(make_request(), drones)


def test_request_that_passes_is_accepted_by_the_planner() -> None:
    # Guards against this validation drifting from what gnc actually accepts.
    from gnc.src.orthomosaic import DroneSpec, ScanRequest, plan_swarm_orthomosaic

    request = make_request()
    validate_request(request, [make_drone()])
    ok, routes = plan_swarm_orthomosaic(
        ScanRequest(
            boundary=request.boundary,
            camera=request.camera,
            target_gsd_m=request.target_gsd_m,
            forward_overlap=request.forward_overlap,
            side_overlap=request.side_overlap,
        ),
        [DroneSpec("drone-01")],
    )
    assert ok
    assert routes is not None and routes["drone-01"]
