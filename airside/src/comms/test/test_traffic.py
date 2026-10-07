from __future__ import annotations

import copy
import json

import pytest
from comms.traffic import (
    TrafficAircraft,
    decode_message,
    parse_other_aircraft,
    parse_traffic,
)


AIRCRAFT = {
    "aircraftIndex": 2,
    "name": "Goose 3",
    "horizontalKeepAway": 30.0,
    "verticalKeepAway": 5.0,
    "position": {
        "lat": 43.4337,
        "lon": -80.5775,
        "altitude": 40.0,
        "speed": 12.5,
        "direction": 270.0,
    },
}


def _traffic_message(*aircraft: dict) -> dict:
    return {
        "event": "traffic",
        "payload": {"siteId": "site-1", "traffic": list(aircraft)},
    }


class TestDecodeMessage:
    def test_decodes_json_object(self) -> None:
        assert decode_message(json.dumps(_traffic_message())) == _traffic_message()

    @pytest.mark.parametrize("raw", ["", "not json", "[1, 2]", "3", None])
    def test_anything_else_is_none(self, raw: str | None) -> None:
        assert decode_message(raw) is None


class TestParseTraffic:
    def test_reads_every_field(self) -> None:
        aircraft, problems = parse_traffic(_traffic_message(AIRCRAFT))

        assert problems == []
        assert aircraft == [
            TrafficAircraft(
                aircraft_index=2,
                name="Goose 3",
                horizontal_keep_away_m=30.0,
                vertical_keep_away_m=5.0,
                lat=43.4337,
                lon=-80.5775,
                altitude_agl_m=40.0,
                speed_mps=12.5,
                direction_deg=270.0,
            )
        ]

    def test_empty_site_has_no_aircraft(self) -> None:
        assert parse_traffic(_traffic_message()) == ([], [])

    def test_whole_numbers_are_accepted(self) -> None:
        entry = copy.deepcopy(AIRCRAFT)
        entry["horizontalKeepAway"] = 30
        entry["position"]["speed"] = 0

        aircraft, problems = parse_traffic(_traffic_message(entry))

        assert problems == []
        assert aircraft[0].horizontal_keep_away_m == 30.0
        assert aircraft[0].speed_mps == 0.0

    @pytest.mark.parametrize(
        "message",
        [{"event": "traffic"}, {"payload": None}, {"payload": {"traffic": "none"}}],
    )
    def test_missing_traffic_list_is_a_problem(self, message: dict) -> None:
        aircraft, problems = parse_traffic(message)

        assert aircraft == []
        assert len(problems) == 1

    def test_bad_entry_does_not_cost_the_others(self) -> None:
        missing_speed = copy.deepcopy(AIRCRAFT)
        del missing_speed["position"]["speed"]
        other = copy.deepcopy(AIRCRAFT)
        other["aircraftIndex"] = 5

        aircraft, problems = parse_traffic(
            _traffic_message(missing_speed, "nonsense", other)
        )

        assert [entry.aircraft_index for entry in aircraft] == [5]
        assert len(problems) == 2

    @pytest.mark.parametrize(
        ("path", "value"),
        [
            (("aircraftIndex",), -1),
            (("aircraftIndex",), 256),
            (("aircraftIndex",), 1.5),
            (("aircraftIndex",), True),
            (("horizontalKeepAway",), "30"),
            (("horizontalKeepAway",), None),
            (("position", "lat"), 91.0),
            (("position", "lon"), -181.0),
            (("position", "direction"), float("nan")),
            (("position", "speed"), float("inf")),
        ],
    )
    def test_nonsense_values_are_skipped(self, path: tuple, value: object) -> None:
        entry = copy.deepcopy(AIRCRAFT)
        target = entry
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value

        aircraft, problems = parse_traffic(_traffic_message(entry))

        assert aircraft == []
        assert len(problems) == 1


def test_parse_other_aircraft_removes_configured_self() -> None:
    own = copy.deepcopy(AIRCRAFT)
    other = copy.deepcopy(AIRCRAFT)
    other["aircraftIndex"] = 5

    aircraft, problems = parse_other_aircraft(
        _traffic_message(own, other), own_aircraft_index=2
    )

    assert problems == []
    assert [item.aircraft_index for item in aircraft] == [5]


def test_parse_other_aircraft_accepts_empty_snapshot() -> None:
    assert parse_other_aircraft(_traffic_message(), own_aircraft_index=2) == ((), [])
