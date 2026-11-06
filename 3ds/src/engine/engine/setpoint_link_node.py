"""
TCP link for the setpoint tester: the ground commands the mission without ROS.

Listens on TCP (parameter ``port``, default ``DEFAULT_PORT``) for the
line-based JSON protocol in ``engine.setpoint_link_protocol`` and republishes
each valid command on ``mission/setpoint`` or ``mission/go_home``, the same
topics the engine subscribes to. Every non-blank line gets a reply so the operator knows
whether the command went out. The groundside client is
``3DS_Groundside/src/setpoint_tester``.
"""

from __future__ import annotations

import socketserver
import threading

import rclpy
import rclpy.node
from airside_interfaces.msg import Coordinate as CoordinateMsg
from engine.setpoint_link_protocol import (
    DEFAULT_PORT,
    MAX_LINE_BYTES,
    GoHome,
    encode_reply,
    parse_command,
)
from std_msgs.msg import Empty


class _ClientHandler(socketserver.StreamRequestHandler):
    """Serves one ground connection: reads command lines, writes replies."""

    server: _LinkServer

    def handle(self) -> None:
        link = self.server.link
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        link.get_logger().info(f"Ground connected from {peer}")
        try:
            while True:
                line = self.rfile.readline(MAX_LINE_BYTES)
                if not line:
                    break
                if len(line) == MAX_LINE_BYTES and not line.endswith(b"\n"):
                    # Discard the rest of the line so the next one parses cleanly.
                    while line and not line.endswith(b"\n"):
                        line = self.rfile.readline(MAX_LINE_BYTES)
                    link.get_logger().warning(f"{peer}: rejected overlong line")
                    self.wfile.write(
                        encode_reply(
                            False, f"rejected: line longer than {MAX_LINE_BYTES} bytes"
                        )
                    )
                    continue
                if not line.strip():
                    continue
                ok, message = link.handle_line(line, peer)
                self.wfile.write(encode_reply(ok, message))
        except OSError as error:
            link.get_logger().warning(f"{peer}: connection error: {error}")
        finally:
            link.get_logger().info(f"Ground disconnected from {peer}")


class _LinkServer(socketserver.ThreadingTCPServer):
    # Rebind immediately when the node respawns.
    allow_reuse_address = True
    # Handler threads blocked on a silent client must not hold up shutdown.
    daemon_threads = True

    def __init__(self, address: tuple[str, int], link: SetpointLink) -> None:
        self.link = link
        super().__init__(address, _ClientHandler)


class SetpointLink(rclpy.node.Node):
    """
    Republishes ground commands received over TCP on the engine's command
    topics.
    """

    SETPOINT_TOPIC = "mission/setpoint"
    GO_HOME_TOPIC = "mission/go_home"

    def __init__(self, **kwargs) -> None:
        super().__init__("setpoint_link", **kwargs)

        host = self.declare_parameter("host", "0.0.0.0").value
        port = self.declare_parameter("port", DEFAULT_PORT).value

        self._setpoint_pub = self.create_publisher(
            msg_type=CoordinateMsg,
            topic=self.SETPOINT_TOPIC,
            qos_profile=10,
        )
        self._go_home_pub = self.create_publisher(
            msg_type=Empty,
            topic=self.GO_HOME_TOPIC,
            qos_profile=10,
        )
        # Handler threads can run concurrently when several clients connect.
        self._publish_lock = threading.Lock()

        # Binds and listens immediately; connections queue until serve_forever.
        self._server = _LinkServer((host, port), self)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, name="setpoint_link_server", daemon=True
        )

        self.get_logger().info(
            f"Setpoint link ready - listening on {host}:{self.port}, forwarding to "
            f"'{self.SETPOINT_TOPIC}' and '{self.GO_HOME_TOPIC}'."
        )
        self._server_thread.start()

    @property
    def port(self) -> int:
        """The bound TCP port (differs from the parameter when it is 0)."""

        return self._server.server_address[1]

    def handle_line(self, line: bytes, peer: str) -> tuple[bool, str]:
        """
        Publishes the command in ``line``. Returns ``(ok, message)`` for the
        reply.
        """

        try:
            command = parse_command(line)
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f"{peer}: rejected command: {error}")
            return False, f"rejected: {error}"

        with self._publish_lock:
            if isinstance(command, GoHome):
                publisher = self._go_home_pub
                publisher.publish(Empty())
                description = "go home"
            else:
                publisher = self._setpoint_pub
                publisher.publish(
                    CoordinateMsg(lat=command.lat, lon=command.lon, alt=command.alt)
                )
                description = f"setpoint {command}"
            subscribers = publisher.get_subscription_count()

        self.get_logger().info(f"{peer}: published {description}")
        message = f"published {description} on {publisher.topic_name}"
        if subscribers == 0:
            message += " - WARNING: nothing is subscribed, is the engine running?"
        return True, message

    def close(self) -> None:
        """Stops accepting connections and closes the listening socket."""

        self._server.shutdown()
        self._server.server_close()
        self._server_thread.join()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SetpointLink()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
