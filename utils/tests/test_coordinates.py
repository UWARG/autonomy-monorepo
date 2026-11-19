import math

import pytest
from shapely.geometry import Polygon

from src.coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
    enu_to_coordinate,
    enu_to_positions,
    local_region,
    positions_to_enu,
    rotate_point,
    rotate_points,
    rotate_polygon,
    validate_polygon,
)
from src.constants import EARTH_RADIUS_M
from src.types import Coordinate
from src.waypoint_utils import east_north_coordinate_offset_m

ANCHOR = Coordinate(43.5, -80.5, 0.0)


def distance_m(a: Coordinate, b: Coordinate) -> float:
    east, north = east_north_coordinate_offset_m(a.lat, a.lon, b.lat, b.lon)
    return math.hypot(east, north)


def test_forward_matches_utils():
    point = Coordinate(43.51, -80.49, 10.0)
    assert coordinate_to_enu(ANCHOR, point) == east_north_coordinate_offset_m(
        ANCHOR.lat, ANCHOR.lon, point.lat, point.lon
    )


@pytest.mark.parametrize(
    "point",
    [
        Coordinate(43.5, -80.5, 0.0),
        Coordinate(43.505, -80.495, 0.0),
        Coordinate(43.55, -80.35, 0.0),
        Coordinate(43.35, -80.8, 0.0),
    ],
)
def test_round_trip_forward_inverse(point):
    east, north = coordinate_to_enu(ANCHOR, point)
    recovered = enu_to_coordinate(ANCHOR, east, north)
    assert distance_m(point, recovered) < 1e-6


@pytest.mark.parametrize("east,north", [(0.0, 0.0), (500.0, 1000.0), (-300.0, 2500.0)])
def test_round_trip_inverse_forward(east, north):
    point = enu_to_coordinate(ANCHOR, east, north)
    east2, north2 = coordinate_to_enu(ANCHOR, point)
    assert math.hypot(east2 - east, north2 - north) < 1e-6


def test_inverse_matches_hand_calculation():
    point = enu_to_coordinate(ANCHOR, east_m=500.0, north_m=1000.0)
    expected_lat = ANCHOR.lat + math.degrees(1000.0 / EARTH_RADIUS_M)
    mean_lat = math.radians((ANCHOR.lat + expected_lat) / 2.0)
    expected_lon = ANCHOR.lon + math.degrees(
        500.0 / (math.cos(mean_lat) * EARTH_RADIUS_M)
    )
    assert point.lat == pytest.approx(expected_lat, abs=1e-12)
    assert point.lon == pytest.approx(expected_lon, abs=1e-12)


def test_north_offset_is_geographic_distance():
    one_kilometer_north = Coordinate(43.5 + 0.009, -80.5, 0.0)
    east, north = coordinate_to_enu(ANCHOR, one_kilometer_north)
    assert north == pytest.approx(0.009 * math.pi / 180.0 * EARTH_RADIUS_M, rel=1e-9)
    assert east == pytest.approx(0.0, abs=1e-9)


def test_anchor_for_boundary_is_centroid():
    boundary = [
        Coordinate(43.5, -80.5, 0.0),
        Coordinate(43.6, -80.3, 0.0),
        Coordinate(43.4, -80.7, 0.0),
    ]
    anchor = anchor_for_boundary(boundary)
    assert anchor.lat == pytest.approx((43.5 + 43.6 + 43.4) / 3.0)
    assert anchor.lon == pytest.approx((-80.5 - 80.3 - 80.7) / 3.0)


def test_boundary_to_enu_matches_per_point():
    boundary = [
        Coordinate(43.51, -80.5, 0.0),
        Coordinate(43.5, -80.45, 0.0),
        Coordinate(43.49, -80.51, 0.0),
    ]
    converted = boundary_to_enu(ANCHOR, boundary)
    assert converted == [
        coordinate_to_enu(ANCHOR, point) for point in boundary
    ]
    assert len(converted) == 3


def test_anchor_for_boundary_rejects_empty():
    with pytest.raises(ValueError):
        anchor_for_boundary([])


def test_enu_to_coordinate_rejects_nonfinite():
    with pytest.raises(ValueError):
        enu_to_coordinate(ANCHOR, float("nan"), 0.0)
    with pytest.raises(ValueError):
        enu_to_coordinate(ANCHOR, 0.0, float("inf"))


def test_rotate_point_ccw_sign_convention():
    east, north = rotate_point((1.0, 0.0), math.pi / 2.0)
    assert east == pytest.approx(0.0, abs=1e-12)
    assert north == pytest.approx(1.0, abs=1e-12)

    east, north = rotate_point((0.0, 1.0), math.pi / 2.0)
    assert east == pytest.approx(-1.0, abs=1e-12)
    assert north == pytest.approx(0.0, abs=1e-12)

    east, north = rotate_point((1.0, 0.0), -math.pi / 2.0)
    assert east == pytest.approx(0.0, abs=1e-12)
    assert north == pytest.approx(-1.0, abs=1e-12)


@pytest.mark.parametrize(
    "point,angle",
    [
        ((1.0, 0.0), 0.3),
        ((0.5, -2.0), 1.1),
        ((-3.0, 4.0), -0.7),
        ((2.0, 1.0), math.pi),
    ],
)
def test_rotate_point_round_trip(point, angle):
    rotated = rotate_point(point, angle)
    recovered = rotate_point(rotated, -angle)
    assert recovered[0] == pytest.approx(point[0], abs=1e-12)
    assert recovered[1] == pytest.approx(point[1], abs=1e-12)


def test_rotate_point_is_isometry():
    a = rotate_point((1.0, 0.0), 0.5)
    b = rotate_point((0.0, 1.0), 0.5)
    assert math.hypot(a[0] - b[0], a[1] - b[1]) == pytest.approx(math.sqrt(2.0))


def test_rotate_points_matches_point_rotation():
    points = [(1.0, 2.0), (-3.0, 0.5), (4.0, -1.0)]
    assert rotate_points(points, 0.7) == [rotate_point(p, 0.7) for p in points]


def test_rotate_polygon_matches_point_rotation():
    polygon = Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)])
    rotated = rotate_polygon(polygon, math.pi / 2.0)
    expected = [rotate_point(p, math.pi / 2.0) for p in polygon.exterior.coords[:-1]]
    for coord, expected_point in zip(rotated.exterior.coords[:-1], expected):
        assert coord[0] == pytest.approx(expected_point[0], abs=1e-9)
        assert coord[1] == pytest.approx(expected_point[1], abs=1e-9)


def test_rotate_polygon_round_trip_preserves_area():
    polygon = Polygon([(0.0, 0.0), (10.0, 0.0), (12.0, 8.0), (2.0, 8.0)])
    rotated = rotate_polygon(polygon, 0.6)
    recovered = rotate_polygon(rotated, -0.6)
    assert recovered.symmetric_difference(polygon).area < 1e-9
    assert recovered.area == pytest.approx(polygon.area)


def test_validate_polygon_rejects_holes():
    ring = Polygon(
        shell=[(0, 0), (10, 0), (10, 10), (0, 10)],
        holes=[[(3, 3), (7, 3), (7, 7), (3, 7)]],
    )
    with pytest.raises(ValueError, match="holes"):
        validate_polygon(ring)


def test_validate_polygon_rejects_self_intersecting():
    bowtie = Polygon([(0, 0), (10, 10), (0, 10), (10, 0)])
    with pytest.raises(ValueError, match="self-intersecting"):
        validate_polygon(bowtie)


def test_validate_polygon_rejects_empty():
    with pytest.raises(ValueError, match="empty"):
        validate_polygon(Polygon())


def test_validate_polygon_rejects_degenerate():
    degenerate = Polygon([(0, 0), (1, 0), (2, 0)])
    with pytest.raises(ValueError):
        validate_polygon(degenerate)

def test_local_region_matches_boundary_conversion():
    boundary = [
        enu_to_coordinate(ANCHOR, east, north)
        for east, north in [(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0)]
    ]
    anchor, region = local_region(boundary)
    assert anchor == anchor_for_boundary(boundary)
    assert region.equals(Polygon(boundary_to_enu(anchor, boundary)))
    assert region.area == pytest.approx(6000.0, rel=1e-5)


def test_local_region_rejects_too_few_vertices():
    with pytest.raises(ValueError, match="3 vertices"):
        local_region([ANCHOR, Coordinate(43.501, -80.5, 0.0)])


def test_local_region_rejects_invalid_polygon():
    bowtie = [
        enu_to_coordinate(ANCHOR, east, north)
        for east, north in [(0.0, 0.0), (10.0, 10.0), (0.0, 10.0), (10.0, 0.0)]
    ]
    with pytest.raises(ValueError, match="self-intersecting"):
        local_region(bowtie)


def test_positions_round_trip_keeps_ids_and_altitude():
    positions = {
        7: enu_to_coordinate(ANCHOR, 12.0, -3.0, 25.0),
        2: enu_to_coordinate(ANCHOR, -40.0, 18.0, 40.0),
    }
    local = positions_to_enu(ANCHOR, positions)
    assert list(local) == [7, 2]
    assert local[7] == pytest.approx((12.0, -3.0), abs=1e-6)
    moved = {drone_id: (east + 5.0, north) for drone_id, (east, north) in local.items()}
    back = enu_to_positions(ANCHOR, moved, positions)
    assert back[7].alt == 25.0
    assert back[2].alt == 40.0
    assert coordinate_to_enu(ANCHOR, back[2]) == pytest.approx((-35.0, 18.0), abs=1e-6)


def test_positions_to_enu_rejects_non_finite():
    with pytest.raises(ValueError, match="finite"):
        positions_to_enu(ANCHOR, {1: Coordinate(math.nan, -80.5, 0.0)})
