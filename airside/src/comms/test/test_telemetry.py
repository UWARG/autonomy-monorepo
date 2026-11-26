import json

from comms.constants import TELEMETRY_SEND_HZ
from comms.telemetry import TelemetryInputs, build_telemetry, send_telemetry


def _inputs(**overrides: object) -> TelemetryInputs:
    values: dict[str, object] = {
        "uav_id": "TEST-UAV",
        "latitude_deg": 43.47,
        "longitude_deg": -80.54,
        "altitude_agl_m": 15.0,
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
    assert reason is None and payload is not None
    assert payload["latitude"] == 43.47
    assert payload["longitude"] == -80.54
    assert payload["altitudeAGL"] == 15.0
    assert payload["uavId"] == "TEST-UAV"
    assert TELEMETRY_SEND_HZ == 1.0


def test_rejects_stale_invalid_or_missing_data() -> None:
    cases = (
        ({"newest_input_age_s": 2.6}, "STALE_TELEMETRY_INPUT"),
        ({"fix_stamp_age_s": 3.0}, "STALE_GPS_MEASUREMENT"),
        ({"latitude_deg": 91.0}, "INVALID_LATITUDE"),
        ({"longitude_deg": -181.0}, "INVALID_LONGITUDE"),
        ({"altitude_agl_m": -1.0}, "INVALID_ALTITUDE"),
        ({"battery_percentage": None}, "MISSING_BATTERY"),
        ({"horizontal_accuracy_m": 0.0}, "INVALID_POSITION_ACCURACY"),
    )
    for overrides, expected_reason in cases:
        payload, reason = build_telemetry(_inputs(**overrides), maximum_input_age_s=2.5)
        assert payload is None and reason == expected_reason


def test_protocol_envelope() -> None:
    class FakeSocket:
        sent: list[str] = []

        def send(self, payload: str) -> None:
            self.sent.append(payload)

    payload, reason = build_telemetry(_inputs(), maximum_input_age_s=2.5)
    assert payload is not None and reason is None
    socket = FakeSocket()
    send_telemetry(socket, payload)
    assert json.loads(socket.sent[0]) == {"action": "telemetry", "data": payload}
