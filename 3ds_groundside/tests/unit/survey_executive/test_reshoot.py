"""
Unit tests for planning reshoot rounds over coverage gaps.
"""

from __future__ import annotations

import pytest
from shapely.geometry import Point, Polygon, box
from utils.src.coordinates import anchor_for_boundary, enu_to_coordinate

from src.groundside_manager.groundside_manager import DroneState
from src.survey_executive import plan_routes
from src.survey_executive.coverage import CoverageMap
from src.survey_executive.planning import plan_waypoints
from src.survey_executive.reshoot import plan_reshoot
from tests.unit.survey_executive.factories import (
    RECTANGLE,
    boundary_enu,
    make_drone,
    make_request,
    nadir_photos,
)

ANCHOR = anchor_for_boundary(RECTANGLE)


def survey_with_dropped_photos(
    drone_count: int,
) -> tuple[CoverageMap, list[DroneState]]:
    """
    Simulated survey where the photos near two spots never arrived.
    """

    request = make_request()
    drones = [make_drone(f"drone-{n:02d}") for n in range(1, drone_count + 1)]
    routes = plan_routes(request, drones, round_index=0)
    boundary = boundary_enu(RECTANGLE)
    min_e, min_n, max_e, max_n = boundary.bounds
    spots = [
        (min_e + 0.25 * (max_e - min_e), min_n + 0.3 * (max_n - min_n)),
        (min_e + 0.75 * (max_e - min_e), min_n + 0.7 * (max_n - min_n)),
    ]
    coverage = CoverageMap(boundary)
    for image_id, footprint in nadir_photos(request, routes).items():
        centre = footprint.centroid
        if all(centre.distance(Point(spot)) > 15.0 for spot in spots):
            coverage.add_photo(image_id, footprint)
    return coverage, drones


@pytest.mark.parametrize("drone_count", [1, 2, 3])
def test_reshooting_the_gaps_restores_full_coverage(drone_count: int) -> None:
    request = make_request()
    coverage, drones = survey_with_dropped_photos(drone_count)
    gaps = coverage.gaps(request.min_gap_area_m2)
    assert gaps and coverage.covered_fraction() < 1.0

    routes, unplanned = plan_reshoot(request, gaps, ANCHOR, drones, round_index=1)
    assert unplanned == []
    for image_id, footprint in nadir_photos(request, routes).items():
        coverage.add_photo(image_id, footprint)

    assert coverage.gaps(request.min_gap_area_m2) == []
    # Float slivers along photo edges still count against the fraction.
    assert coverage.covered_fraction() == pytest.approx(1.0, abs=1e-6)


def test_reshoot_ids_are_per_round_unique_and_consecutive_per_drone() -> None:
    request = make_request()
    drones = [make_drone("drone-01"), make_drone("drone-02")]
    gaps = [box(-40.0, -30.0, -20.0, -10.0), box(15.0, 10.0, 35.0, 35.0)]
    routes, unplanned = plan_reshoot(request, gaps, ANCHOR, drones, round_index=2)

    assert unplanned == []
    assert set(routes) == {"drone-01", "drone-02"}
    all_ids = [m.task_id for route in routes.values() for m in route]
    assert len(all_ids) == len(set(all_ids))
    for drone_id, route in routes.items():
        assert [m.task_id for m in route] == [
            f"survey-1-r2-{drone_id}-{index:03d}" for index in range(len(route))
        ]


def test_each_gap_starts_where_the_previous_one_ended() -> None:
    request = make_request()
    drone = make_drone("drone-01")
    first, second = box(-40.0, -30.0, -20.0, -10.0), box(15.0, 10.0, 35.0, 35.0)
    routes, _ = plan_reshoot(request, [first, second], ANCHOR, [drone], round_index=1)

    def gps(gap: Polygon):
        return [enu_to_coordinate(ANCHOR, e, n) for e, n in list(gap.exterior.coords)[:-1]]

    leg_one = plan_waypoints(request, {"drone-01": None}, gps(first))["drone-01"]
    leg_two = plan_waypoints(
        request, {"drone-01": leg_one[-1][0]}, gps(second)
    )["drone-01"]
    assert [m.target for m in routes["drone-01"]] == [p for p, _ in leg_one + leg_two]
    assert [m.yaw for m in routes["drone-01"]] == [y for _, y in leg_one + leg_two]


def test_gap_with_a_covered_island_is_planned_over_its_outer_edge() -> None:
    request = make_request()
    holed = box(-30.0, -30.0, 30.0, 30.0).difference(box(-5.0, -5.0, 5.0, 5.0))
    assert len(holed.interiors) == 1
    routes, unplanned = plan_reshoot(
        request, [holed], ANCHOR, [make_drone()], round_index=1
    )
    assert unplanned == []

    coverage = CoverageMap(Polygon(holed.exterior))
    for image_id, footprint in nadir_photos(request, routes).items():
        coverage.add_photo(image_id, footprint)
    assert coverage.gaps(request.min_gap_area_m2) == []


def test_unplannable_gap_is_returned_and_the_rest_still_planned() -> None:
    request = make_request()
    bow_tie = Polygon([(0, 0), (20, 20), (20, 0), (0, 20)])
    good = box(-40.0, -30.0, -20.0, -10.0)
    routes, unplanned = plan_reshoot(
        request, [bow_tie, good], ANCHOR, [make_drone()], round_index=1
    )
    assert unplanned == [bow_tie]
    assert routes["drone-01"]


def test_no_drones_leaves_every_gap_unplanned() -> None:
    gaps = [box(-40.0, -30.0, -20.0, -10.0)]
    routes, unplanned = plan_reshoot(make_request(), gaps, ANCHOR, [], round_index=1)
    assert routes == {}
    assert unplanned == gaps


def test_no_gaps_gives_empty_routes() -> None:
    routes, unplanned = plan_reshoot(
        make_request(), [], ANCHOR, [make_drone()], round_index=1
    )
    assert routes == {"drone-01": []}
    assert unplanned == []


def test_invalid_round_and_duplicate_drones_are_rejected() -> None:
    gaps = [box(-40.0, -30.0, -20.0, -10.0)]
    with pytest.raises(ValueError, match="round_index"):
        plan_reshoot(make_request(), gaps, ANCHOR, [make_drone()], round_index=0)
    with pytest.raises(ValueError, match="duplicate"):
        plan_reshoot(
            make_request(), gaps, ANCHOR, [make_drone(), make_drone()], round_index=1
        )


def test_gaps_below_the_minimum_never_reach_the_planner() -> None:
    # Design test plan: gaps below the minimum are skipped. One photo covers
    # all but a 0.5 m2 corner, below the 1 m2 default.
    request = make_request()
    boundary = box(-40.0, -40.0, 40.0, 40.0)
    coverage = CoverageMap(boundary)
    coverage.add_photo("most", boundary.difference(box(39.5, 39.0, 40.0, 40.0)))
    assert [gap.area for gap in coverage.gaps(0.0)] == [pytest.approx(0.5)]

    gaps = coverage.gaps(request.min_gap_area_m2)
    routes, unplanned = plan_reshoot(request, gaps, ANCHOR, [make_drone()], round_index=1)
    assert gaps == []
    assert routes == {"drone-01": []}
    assert unplanned == []
