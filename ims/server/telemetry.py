"""
Builds the AEAC telemetry packet from the drone's latest MAVROS readings.
"""

from __future__ import annotations

import math

from .drone import BATTERY_TOPIC, GLOBAL_POSITION_TOPIC, REL_ALT_TOPIC, STATE_TOPIC

# No message from the drone for this long counts as a lost link.
LINK_TIMEOUT_S = 2.0

# TODO: placeholders until a real source exists for these.
_H_ACCURACY_M = 1.0
_V_ACCURACY_M = 2.0
_RC_LINK_STATUS = 1.0

# ArduPilot modes in which the autopilot, not the pilot, is flying.
_AUTOMATIC_MODES = {
    "AUTO", "GUIDED", "RTL", "SMART_RTL", "LAND", "CIRCLE", "BRAKE", "FOLLOW",
    "ZIGZAG", "AUTO_RTL", "QRTL", "QLAND",
}

# MAV_STATE values reported in mavros_msgs/State.system_status.
_MAV_STATE_CRITICAL = 5
_MAV_STATE_EMERGENCY = 6


def flight_mode(state: dict, link_ok: bool) -> str:
    """Map MAVROS state onto AEAC's mode values."""
    # No data from the drone, or the Pi has lost its flight controller.
    if not link_ok or not state.get("connected", False):
        return "link-lost"
    if state.get("system_status") in (_MAV_STATE_CRITICAL, _MAV_STATE_EMERGENCY):
        return "failsafe"
    if not state.get("armed", False):
        return "idle"
    return "armed-automatic" if state.get("mode") in _AUTOMATIC_MODES else "armed-pilot"


def build_packet(uav_id: str, messages: dict[str, dict], link_ok: bool, now: float) -> dict | None:
    """The packet to send, or None until position, altitude and state have all arrived once."""
    fix = messages.get(GLOBAL_POSITION_TOPIC)
    rel_alt = messages.get(REL_ALT_TOPIC)
    state = messages.get(STATE_TOPIC)
    if fix is None or rel_alt is None or state is None:
        return None

    battery = messages.get(BATTERY_TOPIC)
    # BatteryState.percentage is NaN when unknown; NaN isn't valid JSON.
    percentage = battery.get("percentage") if battery else None
    battery_pct = percentage * 100.0 if isinstance(percentage, (int, float)) and math.isfinite(percentage) else None

    return {
        "uavId": uav_id,
        "unixTime": now,
        "latitude": fix["latitude"],
        "longitude": fix["longitude"],
        "altitudeAGL": rel_alt["data"],
        "horizontalPositionAccuracy": _H_ACCURACY_M,
        "verticalPositionAccuracy": _V_ACCURACY_M,
        "batteryPercentage": battery_pct,
        "mode": flight_mode(state, link_ok),
        "telemetryLinkStatus": 1.0 if link_ok else 0.0,
        "rcLinkStatus": _RC_LINK_STATUS,
    }
