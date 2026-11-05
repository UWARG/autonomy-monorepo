"""Route matching and replay timing checks without a server connection."""

import json

import pytest

from server_traffic_replay import (
    east_north,
    load_capture,
    match_route,
    replay_start_s,
    truth_at,
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
    frames = capture(tmp_path, [10 + 0.5 * index for index in range(45)])
    route = ((43.0, -80.0), (43.0 + 30 / 111_195, -80.0))
    match = match_route(frames, route, own_index=1)
    assert match is not None
    assert match.aircraft_index == 2
    assert match.segment_index == 0
    assert match.centerline_distance_m < 0.1
    assert match.min_interval_s == pytest.approx(1.0)
    assert match.max_interval_s == pytest.approx(1.0)
    assert replay_start_s(match, frames) >= 0.0
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


def test_replay_requires_lead_in_and_sufficient_duration(tmp_path):
    frames = capture(tmp_path, [25] * 10)
    route = ((43.0, -80.0), (43.0 + 30 / 111_195, -80.0))
    match = match_route(frames, route)
    assert match is not None
    with pytest.raises(ValueError, match="lead-in"):
        replay_start_s(match, frames)
