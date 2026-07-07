import math

import pytest
from airside.src.engine.engine.utils.waypoint_parser import (
    Waypoint,
    enu_offset_m,
    parse_waypoints_file,
    sort_clockwise_sweep,
)

# Campus with 10 m offsets
LAT, LON = 43.4717, -80.5414
DLAT_10M = 0.000090
DLON_10M = 0.000124


def _write_waypoints(tmp_path, content):
    path = tmp_path / "waypoints.txt"
    path.write_text(content)
    return path


def test_parse_comma_separated(tmp_path):
    path = _write_waypoints(tmp_path, "43.47, -80.54, 15\n43.48, -80.55, 20\n")
    assert parse_waypoints_file(path) == (
        None,
        [Waypoint(43.47, -80.54, 15.0), Waypoint(43.48, -80.55, 20.0)],
    )


def test_parse_whitespace_separated(tmp_path):
    path = _write_waypoints(tmp_path, "43.47 -80.54 15\n")
    assert parse_waypoints_file(path) == (None, [Waypoint(43.47, -80.54, 15.0)])


def test_parse_skips_comments_and_blank_lines(tmp_path):
    path = _write_waypoints(
        tmp_path, "# a comment\n\n43.47, -80.54, 15  # trailing comment\n\n"
    )
    assert parse_waypoints_file(path) == (None, [Waypoint(43.47, -80.54, 15.0)])


def test_parse_home_line(tmp_path):
    path = _write_waypoints(
        tmp_path, "h 43.47, -80.54, 15\n43.48, -80.55, 15\nH 43.49 -80.56 15\n"
    )
    with pytest.raises(ValueError):
        parse_waypoints_file(path)  # two home lines

    path = _write_waypoints(tmp_path, "43.48, -80.55, 15\nh 43.47, -80.54, 15\n")
    assert parse_waypoints_file(path) == (
        Waypoint(43.47, -80.54, 15.0),
        [Waypoint(43.48, -80.55, 15.0)],
    )


def test_parse_rejects_wrong_arity(tmp_path):
    path = _write_waypoints(tmp_path, "43.47, -80.54\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)

    path = _write_waypoints(tmp_path, "h 43.47, -80.54\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_rejects_non_numeric(tmp_path):
    path = _write_waypoints(tmp_path, "43.47, -80.54, north\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_rejects_out_of_range_coordinates(tmp_path):
    path = _write_waypoints(tmp_path, "91.0, -80.54, 15\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)

    path = _write_waypoints(tmp_path, "43.47, -181.0, 15\n")
    with pytest.raises(ValueError):
        parse_waypoints_file(path)


def test_parse_missing_file(tmp_path):
    with pytest.raises(OSError):
        parse_waypoints_file(tmp_path / "nope.txt")


def test_enu_offset_scale():
    east, north = enu_offset_m(LAT, LON, LAT + DLAT_10M, LON + DLON_10M)
    assert north == pytest.approx(10.0, abs=0.1)
    assert east == pytest.approx(10.0, abs=0.1)


def test_sweep_orders_clockwise_from_north_without_home():
    # Points on the compass around (LAT, LON), which is their centroid
    north = Waypoint(LAT + DLAT_10M, LON, 15)
    east = Waypoint(LAT, LON + DLON_10M, 15)
    south = Waypoint(LAT - DLAT_10M, LON, 15)
    west = Waypoint(LAT, LON - DLON_10M, 15)

    assert sort_clockwise_sweep([west, south, east, north]) == [
        north,
        east,
        south,
        west,
    ]


def test_sweep_starts_from_home_direction_without_flying_home():
    north = Waypoint(LAT + DLAT_10M, LON, 15)
    east = Waypoint(LAT, LON + DLON_10M, 15)
    south = Waypoint(LAT - DLAT_10M, LON, 15)
    west = Waypoint(LAT, LON - DLON_10M, 15)
    home = Waypoint(LAT, LON + 2 * DLON_10M, 15)  # due east of the centroid

    # home orients the sweep (start nearest east) but is not itself flown
    assert sort_clockwise_sweep([west, south, east, north], home=home) == [
        east,
        south,
        west,
        north,
    ]


def test_sweep_home_on_centroid_falls_back_to_north():
    north = Waypoint(LAT + DLAT_10M, LON, 15)
    south = Waypoint(LAT - DLAT_10M, LON, 15)
    home = Waypoint(LAT, LON, 15)  # exactly the centroid: no direction

    assert sort_clockwise_sweep([south, north], home=home) == [north, south]


def test_sweep_breaks_bearing_ties_nearest_first():
    near = Waypoint(LAT + DLAT_10M, LON, 15)
    far = Waypoint(LAT + 4 * DLAT_10M, LON, 15)
    # pulls the centroid off the near/far meridian
    other = Waypoint(LAT - 4 * DLAT_10M, LON + DLON_10M, 15)

    ordered = sort_clockwise_sweep([far, other, near])
    assert ordered.index(near) < ordered.index(far)


def test_sweep_handles_trivial_lists():
    assert sort_clockwise_sweep([]) == []
    single = [Waypoint(LAT, LON, 15)]
    assert sort_clockwise_sweep(single) == single
    home = Waypoint(LAT + DLAT_10M, LON, 15)
    assert sort_clockwise_sweep([], home=home) == []
    assert sort_clockwise_sweep(single, home=home) == single


def test_sweep_full_circle_is_monotonic_in_bearing():
    points = [
        Waypoint(
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
        east, north = enu_offset_m(centroid_lat, centroid_lon, p.lat, p.lon)
        return math.atan2(east, north) % (2 * math.pi)

    bearings = [compass_bearing(p) for p in ordered]
    assert bearings == sorted(bearings)
