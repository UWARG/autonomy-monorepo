"""Listen to AEAC traffic and publish atomic ROS obstacle snapshots."""

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
from airside_interfaces.msg import (
    Coordinate,
    Obstacle,
    ObstacleSnapshotStatus,
    TrafficAircraft as TrafficAircraftMsg,
    TrafficSnapshot,
)
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


def parse_own_aircraft_index(raw: str) -> int:
    """Parse the explicitly configured AEAC identity."""

    try:
        index = int(raw)
    except ValueError as error:
        raise ValueError("own aircraft index must be an integer") from error
    if not 0 <= index <= 255:
        raise ValueError("own aircraft index must fit uint8")
    return index


class TrafficListener(rclpy.node.Node):
    """Publish each server traffic event as one complete, sequenced snapshot."""

    OBSTACLE_TOPIC = "position_controller/obstacle"
    STATUS_TOPIC = "position_controller/obstacle_snapshot"
    BENDY_TRAFFIC_TOPIC = "/aeac/live_traffic"

    def __init__(self) -> None:
        super().__init__("traffic_listener")
        self._obstacle_pub = self.create_publisher(Obstacle, self.OBSTACLE_TOPIC, 10)
        self._status_pub = self.create_publisher(
            ObstacleSnapshotStatus, self.STATUS_TOPIC, 10
        )
        self._bendy_pub = self.create_publisher(
            TrafficSnapshot, self.BENDY_TRAFFIC_TOPIC, 10
        )
        self._deliveries: queue.Queue[SnapshotDelivery] = queue.Queue()
        self._stop = threading.Event()
        self._receiver_thread: threading.Thread | None = None
        # A listener process can be respawned while the controller stays alive.
        # Starting from wall-clock nanoseconds keeps the uint64 sequence
        # increasing across such restarts instead of making the controller
        # reject the restarted stream as old data.
        self._sequence = time.time_ns()
        self._url = os.environ.get(WEBSOCKET_URL_ENV, "").strip() or DEFAULT_WEBSOCKET_URL
        self._token = os.environ.get(CONNECTION_TOKEN_ENV, "").strip()
        try:
            self._own_aircraft_index = parse_own_aircraft_index(
                os.environ.get(OWN_AIRCRAFT_INDEX_ENV, "")
            )
        except ValueError as error:
            self._own_aircraft_index = None
            self.get_logger().error(f"Invalid '{OWN_AIRCRAFT_INDEX_ENV}': {error}")

        self.create_timer(1.0 / PUBLISH_POLL_HZ, self._publish_deliveries)
        if not self._token or self._own_aircraft_index is None:
            self._queue_unhealthy("MISSING_LIVE_AEAC_CONFIGURATION", connected=False)
            return

        self._receiver_thread = threading.Thread(
            target=self._receive_loop, name="traffic_receiver", daemon=True
        )
        self._receiver_thread.start()
        self.get_logger().info(
            f"Traffic listener ready: '{self._url}' -> '{self.OBSTACLE_TOPIC}'"
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
            stamp = self.get_clock().now().to_msg()
            indices: list[int] = []
            if delivery.healthy:
                for aircraft in delivery.aircraft:
                    message = Obstacle()
                    message.header.stamp = stamp
                    message.header.frame_id = "wgs84"
                    message.sequence = delivery.sequence
                    message.aircraft_index = aircraft.aircraft_index
                    message.horizontal_keep_away = aircraft.horizontal_keep_away_m
                    message.vertical_keep_away = aircraft.vertical_keep_away_m
                    message.position = Coordinate(
                        lat=aircraft.lat,
                        lon=aircraft.lon,
                        alt=aircraft.altitude_agl_m,
                    )
                    message.speed = aircraft.speed_mps
                    message.direction = aircraft.direction_deg
                    self._obstacle_pub.publish(message)
                    indices.append(aircraft.aircraft_index)

            status = ObstacleSnapshotStatus()
            status.header.stamp = stamp
            status.header.frame_id = "wgs84"
            status.sequence = delivery.sequence
            status.connected = delivery.connected
            status.healthy = delivery.healthy
            status.reason = delivery.reason
            status.aircraft_indices = indices
            self._status_pub.publish(status)

            # Publish the same complete server event atomically for #181's
            # BendyRuler2D path. The synthetic fixture retains /aeac/traffic.
            snapshot = TrafficSnapshot()
            snapshot.header.stamp = stamp
            snapshot.header.frame_id = "wgs84"
            snapshot.sequence = delivery.sequence
            snapshot.connected = delivery.connected
            snapshot.healthy = delivery.healthy
            snapshot.reason = delivery.reason
            snapshot.own_aircraft_index = (
                self._own_aircraft_index
                if self._own_aircraft_index is not None
                else -1
            )
            if delivery.healthy:
                for aircraft in delivery.aircraft:
                    snapshot.aircraft.append(
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
                    )
            self._bendy_pub.publish(snapshot)

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
