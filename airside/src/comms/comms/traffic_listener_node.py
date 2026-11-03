"""Listen to AEAC traffic and publish each server event as one ROS snapshot."""

from __future__ import annotations

import dataclasses
import os
import queue
import threading
import time
from urllib.parse import quote

import rclpy
import rclpy.node
import websocket
from airside_interfaces.msg import TrafficAircraft as TrafficAircraftMsg
from airside_interfaces.msg import TrafficSnapshot
from comms.constants import (
    CONNECT_TIMEOUT_S,
    CONNECTION_TOKEN_ENV,
    DEFAULT_WEBSOCKET_URL,
    OWN_AIRCRAFT_INDEX_ENV,
    PUBLISH_POLL_HZ,
    RECEIVE_TIMEOUT_S,
    RECONNECT_DELAY_S,
    WEBSOCKET_URL_ENV,
)
from comms.traffic import (
    ERROR_EVENT,
    TRAFFIC_EVENT,
    TrafficAircraft,
    decode_message,
    parse_other_aircraft,
)


@dataclasses.dataclass(frozen=True)
class SnapshotDelivery:
    sequence: int
    connected: bool
    healthy: bool
    reason: str
    aircraft: tuple[TrafficAircraft, ...] = ()


def parse_own_aircraft_index(raw: str) -> int | None:
    """
    Parses the AEAC traffic index of this aircraft. Empty means it is not in
    the traffic feed, so nothing is filtered out.
    """

    if not raw.strip():
        return None
    try:
        index = int(raw)
    except ValueError as error:
        raise ValueError("own aircraft index must be an integer") from error
    if not 0 <= index <= 255:
        raise ValueError("own aircraft index must fit uint8")
    return index


class TrafficListener(rclpy.node.Node):
    """Publish each server traffic event as one complete, sequenced snapshot."""

    TRAFFIC_TOPIC = "/aeac/traffic"

    def __init__(self) -> None:
        super().__init__("traffic_listener")
        self._traffic_pub = self.create_publisher(
            TrafficSnapshot, self.TRAFFIC_TOPIC, 10
        )
        self._deliveries: queue.Queue[SnapshotDelivery] = queue.Queue()
        self._stop = threading.Event()
        # Starting from wall-clock nanoseconds keeps the sequence increasing
        # across a respawn, so subscribers do not reject the new stream as old.
        self._sequence = time.time_ns()
        self._url = os.environ.get(WEBSOCKET_URL_ENV, "").strip() or DEFAULT_WEBSOCKET_URL
        self._token = os.environ.get(CONNECTION_TOKEN_ENV, "").strip()
        self._config_valid = True
        try:
            self._own_aircraft_index = parse_own_aircraft_index(
                os.environ.get(OWN_AIRCRAFT_INDEX_ENV, "")
            )
        except ValueError as error:
            self._own_aircraft_index = None
            self._config_valid = False
            self.get_logger().error(f"Invalid '{OWN_AIRCRAFT_INDEX_ENV}': {error}")

        self.create_timer(1.0 / PUBLISH_POLL_HZ, self._publish_deliveries)
        if not self._token or not self._config_valid:
            self.get_logger().error(
                f"'{CONNECTION_TOKEN_ENV}' missing or AEAC config invalid; "
                "publishing unhealthy traffic so avoidance holds"
            )
            # Republish so late subscribers still see why there is no traffic.
            self.create_timer(
                1.0,
                lambda: self._queue_unhealthy(
                    "MISSING_LIVE_AEAC_CONFIGURATION", connected=False
                ),
            )
            return

        threading.Thread(
            target=self._receive_loop, name="traffic_receiver", daemon=True
        ).start()
        self.get_logger().info(
            f"Traffic listener ready: '{self._url}' -> '{self.TRAFFIC_TOPIC}'"
        )

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    def _queue_unhealthy(self, reason: str, *, connected: bool) -> None:
        self._deliveries.put(
            SnapshotDelivery(
                sequence=self._next_sequence(),
                connected=connected,
                healthy=False,
                reason=reason,
            )
        )

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._receive_until_disconnected()
            except (websocket.WebSocketException, OSError) as error:
                self.get_logger().warning(
                    f"Competition server connection lost ({type(error).__name__})"
                )
                self._queue_unhealthy("AEAC_DISCONNECTED", connected=False)
            self._stop.wait(RECONNECT_DELAY_S)

    def _receive_until_disconnected(self) -> None:
        connection = websocket.create_connection(
            f"{self._url}{'&' if '?' in self._url else '?'}"
            f"Authorization={quote(self._token, safe='')}",
            timeout=CONNECT_TIMEOUT_S,
        )
        try:
            connection.settimeout(RECEIVE_TIMEOUT_S)
            self.get_logger().info("Connected to the competition server")
            while not self._stop.is_set():
                raw = connection.recv()
                if not raw:
                    raise websocket.WebSocketConnectionClosedException()
                self._handle_message(raw)
        finally:
            connection.close()

    def _handle_message(self, raw: str | bytes) -> None:
        message = decode_message(raw)
        if message is None:
            self._queue_unhealthy("INVALID_AEAC_MESSAGE", connected=True)
            return
        event = message.get("event")
        if event == TRAFFIC_EVENT:
            filtered, problems = parse_other_aircraft(
                message,
                own_aircraft_index=self._own_aircraft_index,
            )
            self._deliveries.put(
                SnapshotDelivery(
                    sequence=self._next_sequence(),
                    connected=True,
                    healthy=not problems,
                    reason="; ".join(problems),
                    aircraft=filtered,
                )
            )
        elif event == ERROR_EVENT:
            self._queue_unhealthy("AEAC_SERVER_ERROR", connected=True)

    def _publish_deliveries(self) -> None:
        while True:
            try:
                delivery = self._deliveries.get_nowait()
            except queue.Empty:
                return
            snapshot = TrafficSnapshot()
            snapshot.header.stamp = self.get_clock().now().to_msg()
            snapshot.header.frame_id = "wgs84"
            snapshot.sequence = delivery.sequence
            snapshot.connected = delivery.connected
            snapshot.healthy = delivery.healthy
            snapshot.reason = delivery.reason
            snapshot.own_aircraft_index = (
                -1 if self._own_aircraft_index is None else self._own_aircraft_index
            )
            if delivery.healthy:
                snapshot.aircraft = [
                    TrafficAircraftMsg(
                        aircraft_index=aircraft.aircraft_index,
                        name=aircraft.name,
                        latitude_deg=aircraft.lat,
                        longitude_deg=aircraft.lon,
                        altitude_agl_m=aircraft.altitude_agl_m,
                        speed_mps=aircraft.speed_mps,
                        heading_deg_true=aircraft.direction_deg,
                        horizontal_keepaway_m=aircraft.horizontal_keep_away_m,
                        vertical_keepaway_m=aircraft.vertical_keep_away_m,
                    )
                    for aircraft in delivery.aircraft
                ]
            self._traffic_pub.publish(snapshot)

    def destroy_node(self) -> None:
        self._stop.set()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrafficListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
