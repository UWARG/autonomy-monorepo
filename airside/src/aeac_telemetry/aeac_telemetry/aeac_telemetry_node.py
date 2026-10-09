"""
Sends vehicle telemetry from MAVROS directly to the AEAC competition server.
"""

from __future__ import annotations

import json
import time
from urllib.parse import quote

import rclpy
import rclpy.executors
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, String
from websockets.exceptions import ConnectionClosed, InvalidHandshake
from websockets.sync.client import connect as ws_connect

_DEFAULT_AEAC_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"

_GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_STATE_TOPIC = "mavros/state"
_BATTERY_TOPIC = "mavros/battery"
# Echo of each packet actually sent to AEAC, for the IMS dashboard.
_SENT_TOPIC = "aeac/telemetry_sent"

_SEND_HZ = 1.0  # AEAC penalizes packet gaps > 1.1s and < 0.4s

_H_ACCURACY_M = 3.0
_V_ACCURACY_M = 3.0
_TELEMETRY_LINK_STATUS = 1.0
# TODO: placeholder until a real source exists for RC link status.
_RC_LINK_STATUS = 1.0


class AeacTelemetryNode(Node):
    """Streams position/altitude/battery/mode from MAVROS to AEAC at 1 Hz."""

    def __init__(self) -> None:
        super().__init__("aeac_telemetry_node")

        self.declare_parameter("uav_id", "WARG-01")
        self.declare_parameter("aeac_connection_token", "")
        self.declare_parameter("aeac_websocket_url", _DEFAULT_AEAC_URL)

        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._latest_battery: BatteryState | None = None
        self._ws = None

        self.create_subscription(
            NavSatFix, _GLOBAL_POSITION_TOPIC, self._fix_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float64, _REL_ALT_TOPIC, self._rel_alt_callback, qos_profile_sensor_data
        )
        self.create_subscription(State, _STATE_TOPIC, self._state_callback, 10)
        self.create_subscription(
            BatteryState, _BATTERY_TOPIC, self._battery_callback, qos_profile_sensor_data
        )

        self._sent_publisher = self.create_publisher(String, _SENT_TOPIC, 10)

        self.create_timer(1.0 / _SEND_HZ, self._tick)
        self.get_logger().info("AEAC telemetry node ready - connecting on first tick.")

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._latest_rel_alt_m = msg.data

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def _battery_callback(self, msg: BatteryState) -> None:
        self._latest_battery = msg

    def _connect(self) -> None:
        token = self.get_parameter("aeac_connection_token").value
        if not token:
            self.get_logger().error(
                "aeac_connection_token parameter is empty - set AEAC_CONNECTION_TOKEN",
                throttle_duration_sec=10.0,
            )
            return
        url = self.get_parameter("aeac_websocket_url").value
        separator = "&" if "?" in url else "?"
        full_url = f"{url}{separator}Authorization={quote(token, safe='')}"
        try:
            self._ws = ws_connect(full_url, open_timeout=10)
            self.get_logger().info("connected to AEAC")
        except (InvalidHandshake, OSError) as error:
            self.get_logger().warning(f"AEAC connection failed: {error}; will retry")
            self._ws = None

    def _build_telemetry(self) -> dict | None:
        if self._latest_fix is None or self._latest_rel_alt_m is None or self._latest_state is None:
            return None
        fix_time_s = self._latest_fix.header.stamp.sec + self._latest_fix.header.stamp.nanosec / 1e9
        battery_pct = self._latest_battery.percentage * 100.0 if self._latest_battery else None
        return {
            "uavId": self.get_parameter("uav_id").value,
            "unixTime": fix_time_s if fix_time_s > 0 else time.time(),
            "latitude": self._latest_fix.latitude,
            "longitude": self._latest_fix.longitude,
            "altitudeAGL": self._latest_rel_alt_m,
            "horizontalPositionAccuracy": _H_ACCURACY_M,
            "verticalPositionAccuracy": _V_ACCURACY_M,
            "batteryPercentage": battery_pct,
            "mode": "armed-automatic" if self._latest_state.armed else "idle",
            "telemetryLinkStatus": _TELEMETRY_LINK_STATUS,
            "rcLinkStatus": _RC_LINK_STATUS,
        }

    def _drain_events(self) -> None:
        # Drain the whole queue each tick (AEAC sends ~2 msg/s); timeout=0 never blocks.
        while True:
            try:
                raw = self._ws.recv(timeout=0)
            except TimeoutError:
                return
            try:
                message = json.loads(raw)
            except ValueError:
                continue
            event = message.get("event")
            if event == "infraction":
                self.get_logger().warning(f"AEAC infraction: {message}")
            elif event == "error":
                self.get_logger().warning(f"AEAC error: {message.get('message')}")
            # telemetry_ack and traffic are expected steady-state noise, not logged here.

    def _tick(self) -> None:
        if self._ws is None:
            self._connect()
            if self._ws is None:
                return

        telemetry = self._build_telemetry()
        if telemetry is None:
            self.get_logger().warning(
                f"waiting for '{_STATE_TOPIC}', '{_GLOBAL_POSITION_TOPIC}' and '{_REL_ALT_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return

        try:
            self._ws.send(json.dumps({"action": "telemetry", "data": telemetry}))
            self._sent_publisher.publish(
                String(data=json.dumps({"sent_at": time.time(), "payload": telemetry}))
            )
            self._drain_events()
        except (ConnectionClosed, OSError) as error:
            self.get_logger().warning(f"AEAC connection lost: {error}; will reconnect")
            self._ws = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AeacTelemetryNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
