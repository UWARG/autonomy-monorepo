"""Route matching and replay timing checks without a server connection."""

import json

import pytest

from server_traffic_replay import (
    east_north,
    load_capture,
    match_route,
    mission_path_indices,
    required_replay_duration_s,
    route_length_m,
    truth_at,
    validate_replay_window,
)


def sample(index, lat, lon, speed=1.0):
    return {
        "aircraft_index": index,
        "latitude_deg": lat,
        "longitude_deg": lon,
        "altitude_agl_m": 30,
        "speed_mps": speed,
        "heading_deg_true": 90,
        "horizontal_keepaway_m": 5,
        "vertical_keepaway_m": 5,
    }


def capture(tmp_path, positions):
    path = tmp_path / "capture.jsonl"
    path.write_text(
        "\n".join(
            json.dumps(
                {
                    "received_monotonic_s": 100 + index,
                    "aircraft": [sample(2, 43.0 + north / 111_195, -80.0)],
                    "problems": [],
                }
            )
            for index, north in enumerate(positions)
        ),
        encoding="utf-8",
    )
    return load_capture(path)


def test_matches_moving_aircraft_on_route_and_preserves_timing(tmp_path):
    frames = capture(tmp_path, [10 + 0.5 * index for index in range(70)])
    route = ((43.0, -80.0), (43.0 + 30 / 111_195, -80.0))
    match = match_route(frames, route, own_index=1)
    assert match is not None
    assert match.aircraft_index == 2
    assert match.segment_index == 0
    assert match.centerline_distance_m < 0.1
    assert match.min_interval_s == pytest.approx(1.0)
    assert match.max_interval_s == pytest.approx(1.0)
    assert validate_replay_window(frames, route, 0.0) == pytest.approx(60.0, abs=0.1)
    midpoint = truth_at(frames, 15.5, 2)
    assert midpoint is not None
    assert east_north(route[0], (midpoint.lat, midpoint.lon))[1] == pytest.approx(
        17.75, abs=0.1
    )


def test_rejects_nonincreasing_snapshots_and_noninteger_index(tmp_path):
    path = tmp_path / "bad.jsonl"
    first = {
        "received_monotonic_s": 100,
        "aircraft": [sample(2, 43, -80)],
        "problems": [],
    }
    path.write_text("\n".join(json.dumps(first) for _ in range(2)), encoding="utf-8")
    with pytest.raises(ValueError, match="non-increasing"):
        load_capture(path)
    first["aircraft"][0]["aircraft_index"] = 2.5
    path.write_text(
        json.dumps(first) + "\n" + json.dumps({**first, "received_monotonic_s": 101}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="non-integer"):
        load_capture(path)


def test_no_route_match_when_traffic_is_far_or_is_our_aircraft(tmp_path):
    frames = capture(tmp_path, [200] * 30)
    route = ((43.0, -80.0), (43.0 + 30 / 111_195, -80.0))
    assert match_route(frames, route, own_index=1) is None
    nearby = capture(tmp_path, [10] * 30)
    assert match_route(nearby, route, own_index=2) is None


def test_replay_requires_enough_data_for_the_complete_route(tmp_path):
    frames = capture(tmp_path, [25] * 10)
    route = ((43.0, -80.0), (43.0 + 30 / 111_195, -80.0))
    match = match_route(frames, route)
    assert match is not None
    assert route_length_m(route) == pytest.approx(60.0, abs=0.1)
    assert required_replay_duration_s(route) == pytest.approx(60.0, abs=0.1)
    with pytest.raises(ValueError, match="full route requires"):
        validate_replay_window(frames, route, 0.0)


def test_match_is_limited_to_preselected_replay_window(tmp_path):
    frames = capture(tmp_path, [200] * 20 + [10] * 50)
    route = ((43.0, -80.0), (43.0 + 30 / 111_195, -80.0))
    assert match_route(frames, route, start_s=0.0, end_s=19.0) is None
    assert match_route(frames, route, start_s=20.0, end_s=69.0) is not None


def test_mission_path_matches_closed_lap_window():
    assert mission_path_indices(3, 1) == ((2, 0, 1, 2, 0), 2)
    assert mission_path_indices(3, 2) == ((0, 1, 2, 0, 1), 2)
    assert mission_path_indices(3, 3) == ((1, 2, 0, 1, 2), 2)
    with pytest.raises(ValueError, match="invalid route entry"):
        mission_path_indices(3, 0)
