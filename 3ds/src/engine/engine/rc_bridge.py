"""
Publishes RC input on ``mavros/rc/in`` parsed straight from MAVLink.
"""

from __future__ import annotations

import rclpy
import rclpy.node
from mavros_msgs.msg import RCIn
from pymavlink import mavutil
from rclpy.qos import qos_profile_sensor_data


class RCBridge(rclpy.node.Node):
    """
    Bridges RC_CHANNELS from the mirrored MAVLink stream to ``mavros/rc/in``.
    """

    RC_IN_TOPIC = "mavros/rc/in"
    DEFAULT_MAVLINK_URL = "udpin:127.0.0.1:14550"
    POLL_PERIOD_S = 0.02

    CHANNEL_FIELDS = tuple(f"chan{i}_raw" for i in range(1, 19))
    UNUSED_CHANNEL_VALUES = (0, 65535)

    def __init__(self) -> None:
        super().__init__("rc_bridge")
        self.declare_parameter("mavlink_url", self.DEFAULT_MAVLINK_URL)

        self._publisher = self.create_publisher(
            msg_type=RCIn,
            topic=self.RC_IN_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )
        self._connection = None
        self._receiving = False
        self._timer = self.create_timer(self.POLL_PERIOD_S, self._poll)

    def _poll(self) -> None:
        if self._connection is None and not self._connect():
            return

        while True:
            message = self._connection.recv_match(
                type="RC_CHANNELS", blocking=False
            )
            if message is None:
                return
            self._publish(message)

    def _connect(self) -> bool:
        url = self.get_parameter("mavlink_url").value
        try:
            self._connection = mavutil.mavlink_connection(url)
        except OSError as error:
            self.get_logger().warning(
                f"Cannot open MAVLink endpoint '{url}': {error}",
                throttle_duration_sec=5.0,
            )
            return False
        self.get_logger().info(f"Listening for RC_CHANNELS on '{url}'")
        return True

    def _publish(self, message) -> None:
        channels = [getattr(message, field) for field in self.CHANNEL_FIELDS]
        while channels and channels[-1] in self.UNUSED_CHANNEL_VALUES:
            channels.pop()

        rc_in = RCIn()
        rc_in.header.stamp = self.get_clock().now().to_msg()
        rc_in.rssi = message.rssi
        rc_in.channels = channels
        self._publisher.publish(rc_in)

        if not self._receiving:
            self._receiving = True
            self.get_logger().info(
                f"Receiving RC_CHANNELS ({len(channels)} channels)"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RCBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
