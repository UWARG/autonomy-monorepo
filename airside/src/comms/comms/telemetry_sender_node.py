"""Send this vehicle's WGS84 position and AGL altitude to AEAC at 1 Hz."""

from __future__ import annotations

import math
import os
import time
from urllib.parse import quote

import rclpy
import websocket
from comms.constants import (
    CONNECT_TIMEOUT_S,
    CONNECTION_TOKEN_ENV,
    DEFAULT_WEBSOCKET_URL,
    MIN_TELEMETRY_INTERVAL_S,
    TELEMETRY_INPUT_FRESHNESS_S,
    TELEMETRY_POLL_HZ,
    UAV_ID_ENV,
    WEBSOCKET_URL_ENV,
)
from comms.telemetry import TelemetryInputs, build_telemetry, send_telemetry
from comms.traffic import decode_message
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float64


class TelemetrySender(Node):
    """Maintain a sender-only AEAC socket with a strict 1 Hz timer."""

    def __init__(self) -> None:
        super().__init__("aeac_telemetry_sender")
        self._url = os.environ.get(WEBSOCKET_URL_ENV, "").strip() or DEFAULT_WEBSOCKET_URL
        self._token = os.environ.get(CONNECTION_TOKEN_ENV, "").strip()
        self._uav_id = os.environ.get(UAV_ID_ENV, "").strip()
        self._connection: websocket.WebSocket | None = None
        self._fix: NavSatFix | None = None
        self._altitude: Float64 | None = None
        self._state: State | None = None
        self._battery: BatteryState | None = None
        self._fix_received_s: float | None = None
        self._altitude_received_s: float | None = None
        self._state_received_s: float | None = None
        self._last_ack_s: float | None = None
        self._last_send_s: float | None = None
        self._sent_count = 0
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "aeac/telemetry_diagnostics", 10
        )

        self.create_subscription(
            NavSatFix,
            "mavros/global_position/global",
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            "mavros/global_position/rel_alt",
            self._altitude_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(State, "mavros/state", self._state_callback, 10)
        self.create_subscription(
            BatteryState,
            "mavros/battery",
            self._battery_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / TELEMETRY_POLL_HZ, self._tick)

    def _fix_callback(self, message: NavSatFix) -> None:
        self._fix = message
        self._fix_received_s = time.monotonic()

    def _altitude_callback(self, message: Float64) -> None:
        self._altitude = message
        self._altitude_received_s = time.monotonic()

    def _state_callback(self, message: State) -> None:
        self._state = message
        self._state_received_s = time.monotonic()

    def _battery_callback(self, message: BatteryState) -> None:
        self._battery = message

    def _disconnect(self) -> None:
        if self._connection is not None:
            try:
                self._connection.close()
            except OSError:
                pass
        self._connection = None

    def _connect(self) -> bool:
        if not self._token or not self._uav_id:
            self.get_logger().error(
                f"'{CONNECTION_TOKEN_ENV}' and '{UAV_ID_ENV}' are required",
                throttle_duration_sec=10.0,
            )
            return False
        separator = "&" if "?" in self._url else "?"
        try:
            self._connection = websocket.create_connection(
                f"{self._url}{separator}Authorization={quote(self._token, safe='')}",
                timeout=CONNECT_TIMEOUT_S,
            )
            self._connection.settimeout(0.01)
        except (OSError, websocket.WebSocketException) as error:
            self._connection = None
            self.get_logger().warning(
                f"AEAC telemetry connection failed ({type(error).__name__})",
                throttle_duration_sec=5.0,
            )
            return False
        self.get_logger().info("AEAC telemetry sender connected")
        return True

    def _payload(self) -> tuple[dict[str, object] | None, str | None]:
        if (
            self._fix is None
            or self._altitude is None
            or self._state is None
            or self._fix_received_s is None
            or self._altitude_received_s is None
            or self._state_received_s is None
        ):
            return None, "WAITING_FOR_MAVROS"
        if self._fix.status.status < NavSatStatus.STATUS_FIX:
            return None, "NO_GPS_FIX"
        if self._battery is None or not math.isfinite(self._battery.percentage):
            return None, "MISSING_BATTERY"
        if not 0.0 <= self._battery.percentage <= 1.0:
            return None, "INVALID_BATTERY"
        now_s = time.monotonic()
        newest_age_s = max(
            now_s - self._fix_received_s,
            now_s - self._altitude_received_s,
            now_s - self._state_received_s,
        )
        stamp_s = self._fix.header.stamp.sec + self._fix.header.stamp.nanosec / 1e9
        covariance = self._fix.position_covariance
        if self._fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None, "MISSING_GPS_ACCURACY"
        if covariance[0] <= 0.0 or covariance[4] <= 0.0 or covariance[8] <= 0.0:
            return None, "INVALID_GPS_ACCURACY"
        horizontal_accuracy_m = math.sqrt(max(covariance[0], covariance[4]))
        vertical_accuracy_m = math.sqrt(covariance[8])
        mode = (
            "armed-automatic"
            if self._state.armed and self._state.mode in {"AUTO", "GUIDED"}
            else "armed-pilot" if self._state.armed else "idle"
        )
        return build_telemetry(
            TelemetryInputs(
                uav_id=self._uav_id,
                latitude_deg=self._fix.latitude,
                longitude_deg=self._fix.longitude,
                altitude_agl_m=self._altitude.data,
                armed=self._state.armed,
                fix_unix_s=stamp_s,
                battery_percentage=self._battery.percentage * 100.0,
                flight_mode=mode,
                horizontal_accuracy_m=horizontal_accuracy_m,
                vertical_accuracy_m=vertical_accuracy_m,
                newest_input_age_s=newest_age_s,
                fix_stamp_age_s=time.time() - stamp_s,
            ),
            maximum_input_age_s=TELEMETRY_INPUT_FRESHNESS_S,
        )

    def _tick(self) -> None:
        # rclpy may execute an overdue timer callback immediately after a
        # delayed one. Never turn that catch-up into a too-fast packet.
        if (
            self._last_send_s is not None
            and time.monotonic() - self._last_send_s < MIN_TELEMETRY_INTERVAL_S
        ):
            return
        payload, reason = self._payload()
        if payload is None:
            self.get_logger().warning(
                f"AEAC telemetry not sent: {reason}", throttle_duration_sec=5.0
            )
            self._publish_diagnostic(reason or "INVALID_TELEMETRY", ready=False)
            return
        if self._connection is None and not self._connect():
            self._publish_diagnostic("AEAC_DISCONNECTED", ready=False)
            return
        try:
            assert self._connection is not None
            send_telemetry(self._connection, payload)
            sent_s = time.monotonic()
            interval_s = None if self._last_send_s is None else sent_s - self._last_send_s
            self._last_send_s = sent_s
            self._sent_count += 1
            server_error = self._drain_events()
            ack_recent = (
                self._last_ack_s is not None
                and sent_s - self._last_ack_s <= TELEMETRY_INPUT_FRESHNESS_S
            )
            self._publish_diagnostic(
                server_error or ("AEAC_ACK_RECENT" if ack_recent else "WAITING_FOR_AEAC_ACK"),
                ready=server_error is None and ack_recent,
                interval_s=interval_s,
            )
        except (OSError, websocket.WebSocketException) as error:
            self.get_logger().warning(
                f"AEAC telemetry connection lost ({type(error).__name__})"
            )
            self._last_ack_s = None
            self._publish_diagnostic("AEAC_DISCONNECTED", ready=False)
            self._disconnect()

    def _drain_events(self) -> str | None:
        """Drain a bounded batch so traffic events cannot backlog behind ACKs."""

        assert self._connection is not None
        server_error = None
        for _ in range(20):
            try:
                raw = self._connection.recv()
            except websocket.WebSocketTimeoutException:
                break
            if not raw:
                raise websocket.WebSocketConnectionClosedException()
            message = decode_message(raw)
            if message is None:
                server_error = "INVALID_AEAC_MESSAGE"
            elif message.get("event") == "telemetry_ack":
                self._last_ack_s = time.monotonic()
            elif message.get("event") == "error":
                server_error = "AEAC_SERVER_ERROR"
                self.get_logger().error("AEAC server returned a telemetry error")
            elif message.get("event") == "infraction":
                server_error = "AEAC_INFRACTION"
                self.get_logger().error("AEAC server reported a telemetry infraction")
        return server_error

    def _publish_diagnostic(
        self, reason: str, *, ready: bool, interval_s: float | None = None
    ) -> None:
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "aeac/telemetry_sender"
        status.hardware_id = "airside"
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.ERROR
        status.message = reason
        status.values = [
            KeyValue(key="sent_count", value=str(self._sent_count)),
            KeyValue(
                key="last_send_interval_s",
                value="" if interval_s is None else f"{interval_s:.3f}",
            ),
        ]
        diagnostic.status = [status]
        self._diagnostics_pub.publish(diagnostic)

    def destroy_node(self) -> None:
        self._disconnect()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TelemetrySender()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
