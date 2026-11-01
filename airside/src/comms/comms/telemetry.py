"""Pure construction and validation of AEAC telemetry messages."""

from __future__ import annotations

import dataclasses
import json
import math
from typing import Protocol


class WebSocketSender(Protocol):
    def send(self, payload: str) -> object: ...


@dataclasses.dataclass(frozen=True)
class TelemetryInputs:
    uav_id: str
    latitude_deg: float
    longitude_deg: float
    altitude_agl_m: float
    armed: bool
    fix_unix_s: float
    battery_percentage: float | None
    flight_mode: str
    horizontal_accuracy_m: float
    vertical_accuracy_m: float
    newest_input_age_s: float
    fix_stamp_age_s: float


def build_telemetry(
    inputs: TelemetryInputs, *, maximum_input_age_s: float
) -> tuple[dict[str, object] | None, str | None]:
    """Build one AEAC telemetry payload, or explain why it is unsafe to send."""

    if not inputs.uav_id.strip():
        return None, "MISSING_UAV_ID"
    numeric = (
        inputs.latitude_deg,
        inputs.longitude_deg,
        inputs.altitude_agl_m,
        inputs.fix_unix_s,
        inputs.newest_input_age_s,
        inputs.fix_stamp_age_s,
        inputs.horizontal_accuracy_m,
        inputs.vertical_accuracy_m,
    )
    if not all(math.isfinite(value) for value in numeric):
        return None, "NONFINITE_TELEMETRY"
    if not -90.0 <= inputs.latitude_deg <= 90.0:
        return None, "INVALID_LATITUDE"
    if not -180.0 <= inputs.longitude_deg <= 180.0:
        return None, "INVALID_LONGITUDE"
    if inputs.newest_input_age_s < 0.0 or inputs.newest_input_age_s > maximum_input_age_s:
        return None, "STALE_TELEMETRY_INPUT"
    if inputs.fix_unix_s <= 0.0 or not -1.0 <= inputs.fix_stamp_age_s <= maximum_input_age_s:
        return None, "STALE_GPS_MEASUREMENT"
    if inputs.battery_percentage is None:
        return None, "MISSING_BATTERY"
    if not math.isfinite(inputs.battery_percentage) or not 0.0 <= inputs.battery_percentage <= 100.0:
        return None, "INVALID_BATTERY"
    if inputs.horizontal_accuracy_m <= 0.0 or inputs.vertical_accuracy_m <= 0.0:
        return None, "INVALID_POSITION_ACCURACY"
    if inputs.flight_mode not in {
        "off", "idle", "link-lost", "failsafe", "armed-pilot", "armed-automatic"
    }:
        return None, "INVALID_FLIGHT_MODE"

    return {
        "uavId": inputs.uav_id,
        "unixTime": inputs.fix_unix_s,
        "latitude": inputs.latitude_deg,
        "longitude": inputs.longitude_deg,
        "altitudeAGL": inputs.altitude_agl_m,
        "horizontalPositionAccuracy": inputs.horizontal_accuracy_m,
        "verticalPositionAccuracy": inputs.vertical_accuracy_m,
        "batteryPercentage": inputs.battery_percentage,
        "mode": inputs.flight_mode,
        "telemetryLinkStatus": 1.0,
        "rcLinkStatus": 1.0,
    }, None


def send_telemetry(socket: WebSocketSender, payload: dict[str, object]) -> None:
    """Send one protocol-correct telemetry action through a socket-like object."""

    socket.send(json.dumps({"action": "telemetry", "data": payload}))
