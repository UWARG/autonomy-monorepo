from __future__ import annotations

import json
from pathlib import Path

import pytest
from aeac_bridge.protocol import TrafficProtocolError, parse_traffic_event

FIXTURE = Path(__file__).parent / "fixtures" / "synthetic_traffic_event.json"


def test_pr175_schema_fixture_is_parsed_strictly() -> None:
    event = parse_traffic_event(FIXTURE.read_text(encoding="utf-8"))

    assert event is not None
    assert len(event.aircraft) == 1
    aircraft = event.aircraft[0]
    assert aircraft.aircraft_index == 7
    assert aircraft.name == "SIM-INTRUDER"
    assert aircraft.latitude_deg == pytest.approx(43.4718)
    assert aircraft.longitude_deg == pytest.approx(-80.5415)
    assert aircraft.altitude_agl_m == pytest.approx(15.0)
    assert aircraft.speed_mps == pytest.approx(4.0)
    assert aircraft.heading_deg_true == pytest.approx(90.0)
    assert aircraft.horizontal_keepaway_m == pytest.approx(10.0)
    assert aircraft.vertical_keepaway_m == pytest.approx(5.0)


def test_fresh_empty_traffic_is_valid_clear_data() -> None:
    event = parse_traffic_event('{"event":"traffic","payload":{"traffic":[]}}')

    assert event is not None
    assert event.aircraft == ()


def test_non_traffic_events_are_accepted_but_not_converted() -> None:
    assert parse_traffic_event('{"event":"telemetry_ack"}') is None


@pytest.mark.parametrize(
    "mutator",
    [
        lambda message: message.pop("event"),
        lambda message: message["payload"].update({"traffic": {}}),
        lambda message: message["payload"]["traffic"][0].update({"aircraftIndex": -1}),
        lambda message: message["payload"]["traffic"][0]["position"].update(
            {"lat": 91.0}
        ),
        lambda message: message["payload"]["traffic"][0]["position"].update(
            {"speed": -0.1}
        ),
        lambda message: message["payload"]["traffic"][0]["position"].update(
            {"direction": 360.0}
        ),
        lambda message: message["payload"]["traffic"][0].update(
            {"horizontalKeepAway": 0.0}
        ),
    ],
)
def test_malformed_or_unsafe_traffic_fails_closed(mutator) -> None:
    message = json.loads(FIXTURE.read_text(encoding="utf-8"))
    mutator(message)

    with pytest.raises(TrafficProtocolError):
        parse_traffic_event(json.dumps(message))


def test_duplicate_aircraft_identity_is_rejected() -> None:
    message = json.loads(FIXTURE.read_text(encoding="utf-8"))
    message["payload"]["traffic"].append(message["payload"]["traffic"][0])

    with pytest.raises(TrafficProtocolError, match="unique"):
        parse_traffic_event(json.dumps(message))


def test_invalid_json_is_rejected_without_echoing_payload() -> None:
    with pytest.raises(TrafficProtocolError, match="valid JSON"):
        parse_traffic_event("{secret-invalid-json")
