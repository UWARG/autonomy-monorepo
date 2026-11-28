"""
Unit tests for the live coverage map.
"""

from __future__ import annotations

import pytest
from shapely.geometry import Point, Polygon, box
from utils.src.types import Coordinate

from src.survey_executive import plan_routes
from src.survey_executive.coverage import CoverageMap
from tests.unit.survey_executive.factories import (
    LAT0,
    LON0,
    RECTANGLE,
    boundary_enu,
    make_drone,
    make_request,
    nadir_photos,
)

SQUARE_10 = box(0.0, 0.0, 10.0, 10.0)
QUADRANTS = {
    "sw": box(0.0, 0.0, 5.0, 5.0),
    "se": box(5.0, 0.0, 10.0, 5.0),
    "nw": box(0.0, 5.0, 5.0, 10.0),
    "ne": box(5.0, 5.0, 10.0, 10.0),
}
TRIANGLE = [
    Coordinate(LAT0, LON0, 0.0),
    Coordinate(LAT0, LON0 + 0.003, 0.0),
    Coordinate(LAT0 + 0.0025, LON0 + 0.001, 0.0),
]
L_SHAPE = [
    Coordinate(LAT0, LON0, 0.0),
    Coordinate(LAT0, LON0 + 0.003, 0.0),
    Coordinate(LAT0 + 0.001, LON0 + 0.003, 0.0),
    Coordinate(LAT0 + 0.001, LON0 + 0.001, 0.0),
    Coordinate(LAT0 + 0.003, LON0 + 0.001, 0.0),
    Coordinate(LAT0 + 0.003, LON0, 0.0),
]


def quadrant_map() -> CoverageMap:
    coverage = CoverageMap(SQUARE_10)
    for image_id, footprint in QUADRANTS.items():
        coverage.add_photo(image_id, footprint)
    return coverage


@pytest.mark.parametrize("boundary", [RECTANGLE, TRIANGLE, L_SHAPE])
@pytest.mark.parametrize("target_gsd_m", [0.01, 0.02])
@pytest.mark.parametrize("drone_count", [1, 3])
def test_planned_survey_with_every_photo_covers_the_area(
    boundary: list[Coordinate], target_gsd_m: float, drone_count: int
) -> None:
    # Simulated: level photos at exactly the planned setpoints.
    request = make_request(boundary=boundary, target_gsd_m=target_gsd_m)
    drones = [make_drone(f"drone-{n:02d}") for n in range(drone_count)]
    routes = plan_routes(request, drones, round_index=0)
    coverage = CoverageMap(boundary_enu(boundary))
    for image_id, footprint in nadir_photos(request, routes).items():
        coverage.add_photo(image_id, footprint)

    assert coverage.covered_fraction() == pytest.approx(1.0, abs=1e-9)
    # Only floating point slivers remain, far below the default gap size.
    assert all(gap.area < 1e-6 for gap in coverage.gaps(0.0))
    assert coverage.gaps(request.min_gap_area_m2) == []


def test_missing_photos_in_a_planned_survey_leave_a_gap_there() -> None:
    request = make_request()
    routes = plan_routes(request, [make_drone()], round_index=0)
    photos = nadir_photos(request, routes)
    boundary = boundary_enu(RECTANGLE)
    # Drop every photo whose centre is near the middle of the area.
    middle = boundary.centroid
    dropped = {
        image_id
        for image_id, footprint in photos.items()
        if footprint.centroid.distance(middle) < 20.0
    }
    assert dropped
    coverage = CoverageMap(boundary)
    for image_id, footprint in photos.items():
        if image_id not in dropped:
            coverage.add_photo(image_id, footprint)

    gaps = coverage.gaps(request.min_gap_area_m2)
    assert len(gaps) == 1
    assert gaps[0].contains(middle)
    assert coverage.covered_fraction() < 1.0


def test_a_missing_photo_leaves_exactly_its_area_uncovered() -> None:
    coverage = CoverageMap(SQUARE_10)
    for image_id in ("sw", "se", "nw"):
        coverage.add_photo(image_id, QUADRANTS[image_id])

    assert coverage.covered_fraction() == pytest.approx(0.75)
    gaps = coverage.gaps(0.0)
    assert len(gaps) == 1
    assert gaps[0].equals(QUADRANTS["ne"])


def test_rejected_photo_does_not_count() -> None:
    coverage = quadrant_map()
    coverage.mark_rejected("ne")

    assert coverage.covered_fraction() == pytest.approx(0.75)
    assert [gap.area for gap in coverage.gaps(0.0)] == [pytest.approx(25.0)]


def test_rejection_that_arrives_before_the_photo_still_counts() -> None:
    coverage = CoverageMap(SQUARE_10)
    coverage.mark_rejected("ne")
    for image_id, footprint in QUADRANTS.items():
        coverage.add_photo(image_id, footprint)

    assert coverage.covered_fraction() == pytest.approx(0.75)


def test_repeated_rejection_and_repeated_photo_are_idempotent() -> None:
    coverage = quadrant_map()
    coverage.mark_rejected("sw")
    coverage.mark_rejected("sw")
    coverage.add_photo("ne", QUADRANTS["ne"])

    assert coverage.covered_fraction() == pytest.approx(0.75)


def test_repeated_image_id_replaces_the_earlier_footprint() -> None:
    coverage = quadrant_map()
    coverage.add_photo("ne", box(5.0, 5.0, 7.5, 10.0))

    assert coverage.covered_fraction() == pytest.approx(87.5 / 100.0)


def test_ground_outside_the_boundary_does_not_count() -> None:
    coverage = CoverageMap(SQUARE_10)
    coverage.add_photo("outside", box(20.0, 20.0, 30.0, 30.0))
    coverage.add_photo("overhang", box(-50.0, -50.0, 5.0, 60.0))

    assert coverage.covered_fraction() == pytest.approx(0.5)
    assert coverage.covered().within(SQUARE_10.buffer(1e-9))


def test_overlapping_photos_never_count_above_one() -> None:
    coverage = quadrant_map()
    coverage.add_photo("big", box(-5.0, -5.0, 15.0, 15.0))
    assert coverage.covered_fraction() == pytest.approx(1.0)
    assert coverage.gaps(0.0) == []


def test_empty_map_has_the_whole_boundary_as_one_gap() -> None:
    coverage = CoverageMap(SQUARE_10)
    assert coverage.covered_fraction() == 0.0
    gaps = coverage.gaps(1.0)
    assert len(gaps) == 1 and gaps[0].equals(SQUARE_10)


def test_gaps_are_filtered_by_area_and_largest_first() -> None:
    coverage = CoverageMap(SQUARE_10)
    # Leaves a 10 x 2 strip in the south and a 10 x 1 strip in the north.
    coverage.add_photo("middle", box(0.0, 2.0, 10.0, 9.0))

    gaps = coverage.gaps(0.0)
    assert [gap.area for gap in gaps] == [pytest.approx(20.0), pytest.approx(10.0)]
    assert gaps[0].bounds == pytest.approx((0.0, 0.0, 10.0, 2.0))
    # Strictly greater than the minimum.
    assert [gap.area for gap in coverage.gaps(10.0)] == [pytest.approx(20.0)]
    assert coverage.gaps(20.0) == []


def test_gap_around_a_covered_island_keeps_the_hole() -> None:
    coverage = CoverageMap(SQUARE_10)
    coverage.add_photo("island", box(4.0, 4.0, 6.0, 6.0))

    gaps = coverage.gaps(0.0)
    assert len(gaps) == 1
    assert len(gaps[0].interiors) == 1
    assert gaps[0].area == pytest.approx(96.0)


def test_cached_coverage_updates_after_each_change() -> None:
    coverage = CoverageMap(SQUARE_10)
    coverage.add_photo("sw", QUADRANTS["sw"])
    assert coverage.covered_fraction() == pytest.approx(0.25)
    coverage.add_photo("se", QUADRANTS["se"])
    assert coverage.covered_fraction() == pytest.approx(0.5)
    coverage.mark_rejected("sw")
    assert coverage.covered_fraction() == pytest.approx(0.25)


def test_invalid_inputs_are_rejected() -> None:
    bow_tie = Polygon([(0, 0), (10, 10), (10, 0), (0, 10)])
    with pytest.raises(ValueError, match="self-intersecting"):
        CoverageMap(bow_tie)
    coverage = CoverageMap(SQUARE_10)
    with pytest.raises(ValueError, match="'bad' is not a valid polygon"):
        coverage.add_photo("bad", bow_tie)
    with pytest.raises(ValueError, match="'empty' is not a valid polygon"):
        coverage.add_photo("empty", Polygon())
    with pytest.raises(ValueError, match="min_area_m2"):
        coverage.gaps(-1.0)


def test_point_just_inside_a_gap_is_not_covered() -> None:
    coverage = quadrant_map()
    coverage.mark_rejected("sw")
    assert not coverage.covered().contains(Point(2.5, 2.5))
    assert coverage.covered().contains(Point(7.5, 7.5))


def test_needle_sliver_along_the_boundary_is_not_a_gap() -> None:
    # A photo edge a hair inside the boundary edge, as float error leaves.
    coverage = CoverageMap(SQUARE_10)
    coverage.add_photo("almost", box(0.0, 0.0, 10.0 - 1e-7, 10.0))
    assert coverage.gaps(0.0) == []


def test_narrow_but_real_strip_is_still_a_gap() -> None:
    coverage = CoverageMap(SQUARE_10)
    coverage.add_photo("most", box(0.0, 0.0, 9.99, 10.0))
    gaps = coverage.gaps(0.0)
    assert len(gaps) == 1
    assert gaps[0].bounds == pytest.approx((9.99, 0.0, 10.0, 10.0))
    assert gaps[0].area == pytest.approx(0.1)


def test_rectangular_gap_keeps_its_exact_corners() -> None:
    coverage = quadrant_map()
    coverage.mark_rejected("ne")
    assert coverage.gaps(0.0)[0].equals(QUADRANTS["ne"])
