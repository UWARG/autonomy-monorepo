"""ROS node for bidirectional AEAC telemetry and traffic."""

from __future__ import annotations

import asyncio
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
import rclpy.executors
from airside_interfaces.msg import TrafficAircraft as TrafficAircraftMsg
from airside_interfaces.msg import TrafficSnapshot
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from aeac_bridge.client import AeacClient
from aeac_bridge.protocol import TrafficEvent

_DEFAULT_AEAC_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"
_TRAFFIC_TOPIC = "/aeac/traffic"
_GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_STATE_TOPIC = "mavros/state"
_BATTERY_TOPIC = "mavros/battery"

_H_ACCURACY_M = 1.0
_V_ACCURACY_M = 2.0
_TELEMETRY_LINK_STATUS = 1.0
_RC_LINK_STATUS = 1.0


@dataclass(frozen=True, slots=True)
class _BridgeEvent:
    kind: str
    value: Any = None


class AeacBridgeNode(Node):
    """Own the AEAC socket and expose complete traffic snapshots to ROS."""

    def __init__(self) -> None:
        super().__init__("aeac_bridge")
        self.declare_parameter("uav_id", "")
        self.declare_parameter("own_aircraft_index", -1)
        self.declare_parameter("protocol_verified", False)
        self.declare_parameter("aeac_connection_token", "")
        self.declare_parameter("aeac_websocket_url", _DEFAULT_AEAC_URL)

        self._uav_id = str(self.get_parameter("uav_id").value)
        self._own_aircraft_index = int(self.get_parameter("own_aircraft_index").value)
        self._protocol_verified = bool(self.get_parameter("protocol_verified").value)
        token = str(self.get_parameter("aeac_connection_token").value)
        url = str(self.get_parameter("aeac_websocket_url").value)

        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._latest_state: State | None = None
        self._latest_battery: BatteryState | None = None
        self._telemetry_lock = threading.Lock()
        self._events: queue.SimpleQueue[_BridgeEvent] = queue.SimpleQueue()
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._sequence = 0
        self._configuration_reason = self._validate_configuration(url, token)

        self._traffic_pub = self.create_publisher(TrafficSnapshot, _TRAFFIC_TOPIC, 10)
        self.create_subscription(
            NavSatFix,
            _GLOBAL_POSITION_TOPIC,
            self._fix_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            _REL_ALT_TOPIC,
            self._rel_alt_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(State, _STATE_TOPIC, self._state_callback, 10)
        self.create_subscription(
            BatteryState,
            _BATTERY_TOPIC,
            self._battery_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._drain_events)
        self.create_timer(1.0, self._publish_configuration_failure)

        if self._configuration_reason is None:
            client = AeacClient(
                base_url=url,
                token=token,
                stop_signal=self._stop,
                telemetry_provider=self._build_telemetry,
                on_connected=lambda: self._events.put(_BridgeEvent("connected")),
                on_disconnected=lambda reason: self._events.put(
                    _BridgeEvent("disconnected", reason)
                ),
                on_traffic=lambda traffic: self._events.put(
                    _BridgeEvent("traffic", traffic)
                ),
                on_protocol_error=lambda reason: self._events.put(
                    _BridgeEvent("protocol_error", reason)
                ),
                on_server_event=lambda event, detail: self._events.put(
                    _BridgeEvent("server_event", (event, detail))
                ),
            )
            self._worker = threading.Thread(
                target=self._run_client,
                args=(client,),
                name="aeac-websocket",
                daemon=False,
            )
            self._worker.start()
            self.get_logger().info(
                "AEAC bridge ready; traffic remains fail-closed until a valid event."
            )
        else:
            self.get_logger().error(
                f"AEAC bridge disabled: {self._configuration_reason}"
            )

    def _validate_configuration(self, url: str, token: str) -> str | None:
        if not url.startswith(("ws://", "wss://")):
            return "INVALID_AEAC_URL"
        if not token:
            return "MISSING_AEAC_TOKEN"
        if not self._uav_id:
            return "MISSING_UAV_ID"
        if self._own_aircraft_index < 0:
            return "UNVERIFIED_SELF_IDENTITY"
        if not self._protocol_verified:
            return "PROTOCOL_NOT_VERIFIED"
        return None

    def _fix_callback(self, message: NavSatFix) -> None:
        with self._telemetry_lock:
            self._latest_fix = message

    def _rel_alt_callback(self, message: Float64) -> None:
        with self._telemetry_lock:
            self._latest_rel_alt_m = message.data

    def _state_callback(self, message: State) -> None:
        with self._telemetry_lock:
            self._latest_state = message

    def _battery_callback(self, message: BatteryState) -> None:
        with self._telemetry_lock:
            self._latest_battery = message

    def _build_telemetry(self) -> dict[str, Any] | None:
        with self._telemetry_lock:
            fix = self._latest_fix
            rel_alt_m = self._latest_rel_alt_m
            state = self._latest_state
            battery = self._latest_battery

        if (
            fix is None
            or rel_alt_m is None
            or state is None
            or not state.connected
            or not state.armed
            or fix.status.status < NavSatStatus.STATUS_FIX
        ):
            return None
        numeric = (fix.latitude, fix.longitude, rel_alt_m)
        if not all(math.isfinite(value) for value in numeric):
            return None

        fix_time_s = fix.header.stamp.sec + fix.header.stamp.nanosec / 1e9
        battery_pct = None
        if (
            battery is not None
            and math.isfinite(battery.percentage)
            and 0.0 <= battery.percentage <= 1.0
        ):
            battery_pct = battery.percentage * 100.0
        return {
            "uavId": self._uav_id,
            "unixTime": fix_time_s if fix_time_s > 0.0 else time.time(),
            "latitude": fix.latitude,
            "longitude": fix.longitude,
            "altitudeAGL": rel_alt_m,
            "horizontalPositionAccuracy": _H_ACCURACY_M,
            "verticalPositionAccuracy": _V_ACCURACY_M,
            "batteryPercentage": battery_pct,
            "mode": "armed-automatic",
            "telemetryLinkStatus": _TELEMETRY_LINK_STATUS,
            "rcLinkStatus": _RC_LINK_STATUS,
        }

    def _run_client(self, client: AeacClient) -> None:
        try:
            asyncio.run(client.run())
        # A worker must never die without turning the ROS stream unhealthy.
        except Exception as error:  # noqa: BLE001
            self._events.put(_BridgeEvent("worker_failure", type(error).__name__))

    def _drain_events(self) -> None:
        while True:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                return
            if event.kind == "connected":
                self._publish_status(True, False, "WAITING_FOR_TRAFFIC")
            elif event.kind == "disconnected":
                self._publish_status(False, False, f"DISCONNECTED:{event.value}")
            elif event.kind == "protocol_error":
                self.get_logger().error(f"Rejected AEAC event: {event.value}")
                self._publish_status(True, False, "TRAFFIC_PROTOCOL_ERROR")
            elif event.kind == "worker_failure":
                self.get_logger().error(f"AEAC worker failed: {event.value}")
                self._publish_status(False, False, "WORKER_FAILURE")
            elif event.kind == "server_event":
                server_event, detail = event.value
                log = (
                    self.get_logger().error
                    if server_event == "error"
                    else self.get_logger().warning
                )
                log(f"AEAC {server_event}: {detail}")
                if server_event == "error":
                    self._publish_status(True, False, "AEAC_SERVER_ERROR")
            elif event.kind == "traffic":
                self._sequence += 1
                self._publish_traffic(event.value)

    def _publish_configuration_failure(self) -> None:
        if self._configuration_reason is not None:
            self._publish_status(False, False, self._configuration_reason)

    def _new_snapshot(
        self, *, connected: bool, healthy: bool, reason: str
    ) -> TrafficSnapshot:
        message = TrafficSnapshot()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "wgs84"
        message.sequence = self._sequence
        message.connected = connected
        message.healthy = healthy
        message.reason = reason
        message.own_aircraft_index = self._own_aircraft_index
        return message

    def _publish_status(self, connected: bool, healthy: bool, reason: str) -> None:
        self._traffic_pub.publish(
            self._new_snapshot(connected=connected, healthy=healthy, reason=reason)
        )

    def _publish_traffic(self, event: TrafficEvent) -> None:
        message = self._new_snapshot(connected=True, healthy=True, reason="")
        for aircraft in event.aircraft:
            item = TrafficAircraftMsg()
            item.aircraft_index = aircraft.aircraft_index
            item.name = aircraft.name
            item.latitude_deg = aircraft.latitude_deg
            item.longitude_deg = aircraft.longitude_deg
            item.altitude_agl_m = aircraft.altitude_agl_m
            item.speed_mps = aircraft.speed_mps
            item.heading_deg_true = aircraft.heading_deg_true
            item.horizontal_keepaway_m = aircraft.horizontal_keepaway_m
            item.vertical_keepaway_m = aircraft.vertical_keepaway_m
            message.aircraft.append(item)
        self._traffic_pub.publish(message)
        if self._sequence == 1:
            self.get_logger().info(
                f"Received first healthy AEAC traffic snapshot "
                f"({len(event.aircraft)} aircraft)."
            )

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=5.0)
            if self._worker.is_alive():
                self.get_logger().error("AEAC worker did not stop within 5 seconds")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AeacBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
