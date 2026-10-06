"""
Listens to the AEAC competition server and republishes the simulated traffic
it reports as obstacles.
"""

from __future__ import annotations

import os
import queue
import threading
from urllib.parse import quote

import rclpy
import rclpy.node
import websocket
from airside_interfaces.msg import Coordinate, Obstacle
from comms.constants import (
    CONNECT_TIMEOUT_S,
    CONNECTION_TOKEN_ENV,
    DEFAULT_WEBSOCKET_URL,
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
    parse_traffic,
)


class TrafficListener(rclpy.node.Node):
    """
    Publishes one ``Obstacle`` per aircraft for every traffic snapshot received
    from the competition server.

    The WebSocket is read on its own thread, which reconnects whenever the
    connection drops; snapshots are handed to the ROS side through a queue.
    """

    OBSTACLE_TOPIC = "position_controller/obstacle"

    def __init__(self) -> None:
        super().__init__("traffic_listener")

        self._obstacle_pub = self.create_publisher(
            msg_type=Obstacle,
            topic=self.OBSTACLE_TOPIC,
            qos_profile=10,
        )

        self._snapshots: queue.Queue[list[TrafficAircraft]] = queue.Queue()
        self._stop = threading.Event()
        self._receiver_thread: threading.Thread | None = None

        self._url = os.environ.get(WEBSOCKET_URL_ENV, "").strip() or DEFAULT_WEBSOCKET_URL
        self._token = os.environ.get(CONNECTION_TOKEN_ENV, "").strip()
        if not self._token:
            # Stay up but idle, so a missing token does not crash-loop the launch
            self.get_logger().error(
                f"'{CONNECTION_TOKEN_ENV}' is not set - not connecting to the "
                "competition server, no traffic will be avoided"
            )
            return

        self.create_timer(1.0 / PUBLISH_POLL_HZ, self._publish_snapshots)
        self._receiver_thread = threading.Thread(
            target=self._receive_loop, name="traffic_receiver", daemon=True
        )
        self._receiver_thread.start()

        self.get_logger().info(
            f"Traffic listener ready - forwarding traffic from '{self._url}' "
            f"to '{self.OBSTACLE_TOPIC}'."
        )

    def _receive_loop(self) -> None:
        """
        Keeps a connection to the server open until the node is destroyed.
        """

        while not self._stop.is_set():
            try:
                self._receive_until_disconnected()
            except (websocket.WebSocketException, OSError) as error:
                # Only the error type: its text can contain the URL, and the
                # token with it
                self.get_logger().warning(
                    f"Competition server connection lost ({type(error).__name__}) "
                    f"- reconnecting in {RECONNECT_DELAY_S}s"
                )
            self._stop.wait(RECONNECT_DELAY_S)

    def _receive_until_disconnected(self) -> None:
        # The token travels in the URL, as the server requires
        connection = websocket.create_connection(
            f"{self._url}{'&' if '?' in self._url else '?'}"
            f"Authorization={quote(self._token, safe='')}",
            timeout=CONNECT_TIMEOUT_S,
        )
        try:
            connection.settimeout(RECEIVE_TIMEOUT_S)
            self.get_logger().info("Connected to the competition server.")

            while not self._stop.is_set():
                raw = connection.recv()
                if raw:
                    self._handle_message(raw)
        finally:
            connection.close()

    def _handle_message(self, raw: str | bytes) -> None:
        message = decode_message(raw)
        if message is None:
            self.get_logger().warning(
                "Ignoring message from the competition server that is not a "
                "JSON object",
                throttle_duration_sec=5.0,
            )
            return

        event = message.get("event")
        if event == TRAFFIC_EVENT:
            aircraft, problems = parse_traffic(message)
            for problem in problems:
                self.get_logger().warning(problem, throttle_duration_sec=5.0)
            self._snapshots.put(aircraft)
        elif event == ERROR_EVENT:
            self.get_logger().error(
                f"Competition server error: {message.get('message')}"
            )
        elif event is None:
            # API Gateway's own replies (e.g. a rejected connection) have no event
            self.get_logger().warning(
                f"Unexpected message from the competition server: {message}",
                throttle_duration_sec=5.0,
            )

    def _publish_snapshots(self) -> None:
        while True:
            try:
                snapshot = self._snapshots.get_nowait()
            except queue.Empty:
                return

            for aircraft in snapshot:
                self._obstacle_pub.publish(
                    Obstacle(
                        aircraft_index=aircraft.aircraft_index,
                        horizontal_keep_away=aircraft.horizontal_keep_away_m,
                        position=Coordinate(
                            lat=aircraft.lat,
                            lon=aircraft.lon,
                            alt=aircraft.altitude_agl_m,
                        ),
                        speed=aircraft.speed_mps,
                        direction=aircraft.direction_deg,
                    )
                )

            self.get_logger().debug(f"Published {len(snapshot)} obstacle(s)")

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
