import math

import pytest

from utils.src.types import Coordinate
from utils.src.waypoint_utils import (
    east_north_coordinate_offset_m,
    parse_waypoints_file,
    sort_clockwise_sweep,
)

# Campus with 10 m offsets
LAT, LON = 43.4717, -80.5414
DLAT_10M = 0.000090
DLON_10M = 0.000124


def _write_waypoints(tmp_path, content):
    path = tmp_path / "waypoints.yaml"
    path.write_text(content)
    return path


def test_parse_waypoints(tmp_path):
    path = _write_waypoints(
        tmp_path,
        """
        waypoints:
          - {lat: 43.47, lon: -80.54, alt: 15}
          - {lat: 43.48, lon: -80.55, alt: 20}
        """,
    )
    assert parse_waypoints_file(path) == (
        None,
        [Coordinate(43.47, -80.54, 15.0), Coordinate(43.48, -80.55, 20.0)],
    )


def test_parse_block_style(tmp_path):
    path = _write_waypoints(
        tmp_path,
        """
        waypoints:
          - lat: 43.47
            lon: -80.54
            alt: 15
        """,
    )
    assert parse_waypoints_file(path) == (None, [Coordinate(43.47, -80.54, 15.0)])


def test_parse_skips_comments_and_blank_lines(tmp_path):
    path = _write_waypoints(
        tmp_path,
        """
        # a comment

        waypoints:
          - {lat: 43.47, lon: -80.54, alt: 15}  # trailing comment
        """,
    )
    assert parse_waypoints_file(path) == (None, [Coordinate(43.47, -80.54, 15.0)])


def test_parse_empty_file(tmp_path):
    path = _write_waypoints(tmp_path, "\n# only comments\n")
    assert parse_waypoints_file(path) == (None, [])


def test_parse_home(tmp_path):
    path = _write_waypoints(
        tmp_path,
        """
        home: {lat: 43.47, lon: -80.54, alt: 15}
        waypoints:
          - {lat: 43.48, lon: -80.55, alt: 15}
        """,
    )
    assert parse_waypoints_file(path) == (
        Coordinate(43.47, -80.54, 15.0),
        [Coordinate(43.48, -80.55, 15.0)],
    )


def test_parse_rejects_non_mapping_root(tmp_path):
    path = _write_waypoints(tmp_path, "- 43.47\n- -80.54\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_rejects_missing_keys(tmp_path):
    path = _write_waypoints(
        tmp_path, "waypoints:\n  - {lat: 43.47, lon: -80.54}\n"
    )
    with pytest.raises(ValueError):
        parse_waypoints_file(path)

    path = _write_waypoints(tmp_path, "home: {lat: 43.47, lon: -80.54}\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_rejects_non_numeric(tmp_path):
    path = _write_waypoints(
        tmp_path, "waypoints:\n  - {lat: 43.47, lon: -80.54, alt: north}\n"
    )
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_rejects_out_of_range_coordinates(tmp_path):
    path = _write_waypoints(
        tmp_path, "waypoints:\n  - {lat: 91.0, lon: -80.54, alt: 15}\n"
    )
    with pytest.raises(ValueError):
        parse_waypoints_file(path)

    path = _write_waypoints(
        tmp_path, "waypoints:\n  - {lat: 43.47, lon: -181.0, alt: 15}\n"
    )
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_rejects_invalid_yaml(tmp_path):
    path = _write_waypoints(tmp_path, "waypoints: [{lat: 43.47, lon: -80.54\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_missing_file(tmp_path):
    with pytest.raises(OSError):
        parse_waypoints_file(tmp_path / "nope.yaml")


def test_enu_offset_scale():
    east, north = east_north_coordinate_offset_m(LAT, LON, LAT + DLAT_10M, LON + DLON_10M)
    assert north == pytest.approx(10.0, abs=0.1)
    assert east == pytest.approx(10.0, abs=0.1)


def test_sweep_orders_clockwise_from_north_without_home():
    north = Coordinate(LAT + DLAT_10M, LON, 15)
    east = Coordinate(LAT, LON + DLON_10M, 15)
    south = Coordinate(LAT - DLAT_10M, LON, 15)
    west = Coordinate(LAT, LON - DLON_10M, 15)

    assert sort_clockwise_sweep([west, south, east, north]) == [
        north,
        east,
        south,
        west,
    ]


def test_sweep_starts_from_home_direction_without_flying_home():
    north = Coordinate(LAT + DLAT_10M, LON, 15)
    east = Coordinate(LAT, LON + DLON_10M, 15)
    south = Coordinate(LAT - DLAT_10M, LON, 15)
    west = Coordinate(LAT, LON - DLON_10M, 15)
    home = Coordinate(LAT, LON + 2 * DLON_10M, 15)  # due east of the centroid

    assert sort_clockwise_sweep([west, south, east, north], home=home) == [
        east,
        south,
        west,
        north,
    ]


def test_sweep_home_on_centroid_falls_back_to_north():
    north = Coordinate(LAT + DLAT_10M, LON, 15)
    south = Coordinate(LAT - DLAT_10M, LON, 15)
    home = Coordinate(LAT, LON, 15)  # exactly the centroid: no direction

    assert sort_clockwise_sweep([south, north], home=home) == [north, south]


def test_sweep_breaks_bearing_ties_nearest_first():
    near = Coordinate(LAT + DLAT_10M, LON, 15)
    far = Coordinate(LAT + 4 * DLAT_10M, LON, 15)
    # pulls the centroid off the near/far meridian
    other = Coordinate(LAT - 4 * DLAT_10M, LON + DLON_10M, 15)

    ordered = sort_clockwise_sweep([far, other, near])
    assert ordered.index(near) < ordered.index(far)


def test_sweep_handles_trivial_lists():
    assert sort_clockwise_sweep([]) == []
    single = [Coordinate(LAT, LON, 15)]
    assert sort_clockwise_sweep(single) == single
    home = Coordinate(LAT + DLAT_10M, LON, 15)
    assert sort_clockwise_sweep([], home=home) == []
    assert sort_clockwise_sweep(single, home=home) == single


def test_sweep_full_circle_is_monotonic_in_bearing():
    points = [
        Coordinate(
            LAT + math.cos(angle) * DLAT_10M,
            LON + math.sin(angle) * DLON_10M,
            15,
        )
        for angle in [0.1, 2.0, 4.0, 5.5, 3.1, 0.9]
    ]
    ordered = sort_clockwise_sweep(points)

    centroid_lat = sum(p.lat for p in ordered) / len(ordered)
    centroid_lon = sum(p.lon for p in ordered) / len(ordered)

    def compass_bearing(p):
        east, north = east_north_coordinate_offset_m(centroid_lat, centroid_lon, p.lat, p.lon)
        return math.atan2(east, north) % (2 * math.pi)

    bearings = [compass_bearing(p) for p in ordered]
    assert bearings == sorted(bearings)
