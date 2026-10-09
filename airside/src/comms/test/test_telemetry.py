import json

from comms.constants import TELEMETRY_SEND_HZ
from comms.telemetry import TelemetryInputs, build_telemetry, send_telemetry


def _inputs(**overrides: object) -> TelemetryInputs:
    values: dict[str, object] = {
        "uav_id": "WARG-01",
        "latitude_deg": 43.47,
        "longitude_deg": -80.54,
        "altitude_agl_m": 15.0,
        "armed": True,
        "fix_unix_s": 1_700_000_000.0,
        "battery_percentage": 82.0,
        "flight_mode": "armed-automatic",
        "horizontal_accuracy_m": 1.0,
        "vertical_accuracy_m": 2.0,
        "newest_input_age_s": 0.2,
        "fix_stamp_age_s": 0.2,
    }
    values.update(overrides)
    return TelemetryInputs(**values)  # type: ignore[arg-type]


def test_builds_required_one_hz_payload_fields() -> None:
    payload, reason = build_telemetry(_inputs(), maximum_input_age_s=2.5)

    assert reason is None
    assert payload is not None
    assert payload["latitude"] == 43.47
    assert payload["longitude"] == -80.54
    assert payload["altitudeAGL"] == 15.0
    assert payload["uavId"] == "WARG-01"


def test_rejects_stale_position() -> None:
    payload, reason = build_telemetry(
        _inputs(newest_input_age_s=2.6), maximum_input_age_s=2.5
    )

    assert payload is None
    assert reason == "STALE_TELEMETRY_INPUT"


def test_rejects_republished_old_gps_measurement() -> None:
    payload, reason = build_telemetry(
        _inputs(newest_input_age_s=0.1, fix_stamp_age_s=3.0),
        maximum_input_age_s=2.5,
    )

    assert payload is None
    assert reason == "STALE_GPS_MEASUREMENT"


def test_rejects_invalid_coordinates() -> None:
    payload, reason = build_telemetry(
        _inputs(latitude_deg=91.0), maximum_input_age_s=2.5
    )

    assert payload is None
    assert reason == "INVALID_LATITUDE"


def test_requires_battery_and_accuracy() -> None:
    assert build_telemetry(_inputs(battery_percentage=None), maximum_input_age_s=2.5)[1] == "MISSING_BATTERY"
    assert build_telemetry(_inputs(horizontal_accuracy_m=0.0), maximum_input_age_s=2.5)[1] == "INVALID_POSITION_ACCURACY"


def test_mock_websocket_receives_protocol_envelope() -> None:
    class FakeSocket:
        sent: list[str] = []

        def send(self, payload: str) -> None:
            self.sent.append(payload)

    payload, reason = build_telemetry(_inputs(), maximum_input_age_s=2.5)
    socket = FakeSocket()
    assert payload is not None and reason is None

    send_telemetry(socket, payload)

    assert json.loads(socket.sent[0]) == {"action": "telemetry", "data": payload}
    assert TELEMETRY_SEND_HZ == 1.0
