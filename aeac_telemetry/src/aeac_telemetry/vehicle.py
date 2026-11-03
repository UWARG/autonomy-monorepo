"""Tracks the vehicle from MAVLink messages and builds AEAC telemetry packets."""

from __future__ import annotations

import time
from typing import Any

from pymavlink import mavutil

# A link that has not delivered a HEARTBEAT for this long is lost.
LINK_TIMEOUT_S = 2.0
# RC_CHANNELS older than this means the RC link status is unknown.
RC_TIMEOUT_S = 2.0
# A GLOBAL_POSITION_INT older than this is not a current position.
POSITION_TIMEOUT_S = 2.0
# Reported when there is no valid GPS position.
NO_FIX_LAT_LON = (0.0, 0.0)
NO_FIX_ACCURACY_M = 9999.0

# Flight modes in which the autopilot, not the pilot, is flying (CONOPS
# "armed-automatic").
AUTOMATIC_MODES = frozenset(
    {"AUTO", "GUIDED", "RTL", "LAND", "SMART_RTL", "AUTO_RTL"}
)

_FAILSAFE_STATES = (
    mavutil.mavlink.MAV_STATE_CRITICAL,
    mavutil.mavlink.MAV_STATE_EMERGENCY,
)
_RSSI_UNKNOWN = 255


def flight_mode(*, link_ok: bool, armed: bool, mode: str, system_status: int) -> str:
    """The AEAC flight mode (CONOPS 5.2.3 item 3.b.vii)."""

    if not link_ok:
        return "link-lost"
    if system_status in _FAILSAFE_STATES:
        return "failsafe"
    if not armed:
        return "idle"
    return "armed-automatic" if mode in AUTOMATIC_MODES else "armed-pilot"


class VehicleState:
    """The latest of each MAVLink message that goes into a telemetry packet."""

    def __init__(self) -> None:
        self.heartbeat: Any = None
        self.heartbeat_s = -float("inf")
        self.position: Any = None  # GLOBAL_POSITION_INT
        self.position_s = -float("inf")
        self.gps: Any = None  # GPS_RAW_INT
        self.battery_percent: float | None = None
        self.gps_unix_s: float | None = None  # from SYSTEM_TIME
        self.gps_unix_at_s = 0.0
        self.rc_rssi: int | None = None
        self.rc_s = -float("inf")

    def handle(self, msg: Any, now_s: float) -> None:
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            # Ignore GCSes and peripherals sharing the link.
            if msg.type != mavutil.mavlink.MAV_TYPE_GCS and msg.autopilot != (
                mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                self.heartbeat, self.heartbeat_s = msg, now_s
        elif kind == "GLOBAL_POSITION_INT":
            self.position, self.position_s = msg, now_s
        elif kind == "GPS_RAW_INT":
            self.gps = msg
        elif kind == "SYS_STATUS" and msg.battery_remaining >= 0:
            self.battery_percent = float(msg.battery_remaining)
        elif kind == "SYSTEM_TIME" and msg.time_unix_usec > 0:
            self.gps_unix_s, self.gps_unix_at_s = msg.time_unix_usec / 1e6, now_s
        elif kind == "RC_CHANNELS":
            self.rc_rssi, self.rc_s = msg.rssi, now_s

    def has_fix(self, now_s: float) -> bool:
        """A current 3D GPS fix and a position from it."""

        if self.position is None or now_s - self.position_s > POSITION_TIMEOUT_S:
            return False
        if self.gps is not None and self.gps.fix_type < mavutil.mavlink.GPS_FIX_TYPE_3D_FIX:
            return False
        # ArduPilot reports 0, 0 until its EKF has an origin.
        return (self.position.lat, self.position.lon) != (0, 0)

    def packet(self, uav_id: str, now_s: float) -> dict[str, Any] | None:
        """
        The AEAC telemetry packet, or None until the first heartbeat (without
        one, the armed state is unknown).

        Without a valid fix the packet still goes out, with the no-fix
        position: the server scores that as at most an invalid-GPS infraction
        per 15 s, where silence would be a growing disconnect. It also makes
        the first broadcast idle rather than armed if the FC arms before
        getting a fix.
        """

        if self.heartbeat is None:
            return None
        link_ok = now_s - self.heartbeat_s <= LINK_TIMEOUT_S
        armed = bool(
            self.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        if self.has_fix(now_s):
            lat, lon = self.position.lat / 1e7, self.position.lon / 1e7
            h_acc = self._accuracy_m("h_acc", "eph")
            v_acc = self._accuracy_m("v_acc", "epv")
        else:
            lat, lon = NO_FIX_LAT_LON
            h_acc = v_acc = NO_FIX_ACCURACY_M
        altitude_m = self.position.relative_alt / 1000.0 if self.position else 0.0
        return {
            "uavId": uav_id,
            "unixTime": self._unix_time(now_s),
            "latitude": lat,
            "longitude": lon,
            "altitudeAGL": round(max(0.0, altitude_m), 2),
            "horizontalPositionAccuracy": h_acc,
            "verticalPositionAccuracy": v_acc,
            "batteryPercentage": self.battery_percent or 0.0,
            "mode": flight_mode(
                link_ok=link_ok,
                armed=armed,
                mode=mavutil.mode_string_v10(self.heartbeat),
                system_status=self.heartbeat.system_status,
            ),
            "telemetryLinkStatus": 1.0 if link_ok else 0.0,
            "rcLinkStatus": self._rc_link(now_s),
        }

    def _unix_time(self, now_s: float) -> float:
        # GPS time from the FC, advanced by how long ago it arrived. The
        # system clock is only a fallback before the FC has a GPS time.
        if self.gps_unix_s is None:
            return time.time()
        return round(self.gps_unix_s + (now_s - self.gps_unix_at_s), 3)

    def _accuracy_m(self, accuracy_field: str, dop_field: str) -> float:
        if self.gps is None:
            return NO_FIX_ACCURACY_M
        accuracy_mm = getattr(self.gps, accuracy_field, 0)
        if accuracy_mm:
            return round(accuracy_mm / 1000.0, 2)
        # Older GPS drivers only give DOP (x100); treat it as metres.
        dop = getattr(self.gps, dop_field, 65535)
        return NO_FIX_ACCURACY_M if dop == 65535 else round(dop / 100.0, 2)

    def _rc_link(self, now_s: float) -> float:
        if self.rc_rssi is None or now_s - self.rc_s > RC_TIMEOUT_S:
            return 0.0
        if self.rc_rssi == _RSSI_UNKNOWN:
            return 1.0  # receiver reports channels but not strength
        return round(self.rc_rssi / 254.0, 2)
