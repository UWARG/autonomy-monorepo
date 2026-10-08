from __future__ import annotations

import collections

import py_trees
import rclpy.node
from airside_interfaces.msg import Coordinate as CoordinateMsg
from engine import blackboard_keys
from engine.constants import SETPOINT_MIN_ALTITUDE_M
from engine.ground_log import send_to_ground
from utils.src.types import Coordinate


def setpoint_from_msg(msg: CoordinateMsg) -> Coordinate:
    """
    Converts a setpoint message into a utils Coordinate.

    Raises ``ValueError`` if the latitude/longitude is out of range or the
    altitude is below ``SETPOINT_MIN_ALTITUDE_M``.
    """

    if not (-90.0 <= msg.lat <= 90.0 and -180.0 <= msg.lon <= 180.0):
        raise ValueError(f"latitude/longitude out of range: ({msg.lat}, {msg.lon})")
    if not msg.alt >= SETPOINT_MIN_ALTITUDE_M:
        raise ValueError(
            f"altitude {msg.alt}m is below the {SETPOINT_MIN_ALTITUDE_M}m minimum"
        )
    return Coordinate(lat=msg.lat, lon=msg.lon, alt=msg.alt)


class WaitForSetpoint(py_trees.behaviour.Behaviour):
    """
    Waits for the next setpoint from the ground and stages it for flight.

    Setpoints arriving on ``mission/setpoint`` are queued in order, including
    those received while the drone is busy with an earlier one. Returns
    RUNNING while the queue is empty, then pops the oldest setpoint, writes it
    to ``current_waypoint`` and returns SUCCESS. Invalid setpoints are
    rejected when received.
    """

    SETPOINT_TOPIC = "mission/setpoint"

    def __init__(self, name: str = "WaitForSetpoint") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._pending: collections.deque[Coordinate] = collections.deque()

        self._setpoint_sub = self._node.create_subscription(
            msg_type=CoordinateMsg,
            topic=self.SETPOINT_TOPIC,
            callback=self._setpoint_callback,
            qos_profile=10,
        )

    def _setpoint_callback(self, msg: CoordinateMsg) -> None:
        try:
            setpoint = setpoint_from_msg(msg)
        except ValueError as error:
            self._node.get_logger().error(f"{self.name}: rejected setpoint: {error}")
            send_to_ground(self._node, "ENG: setpoint rejected")
            return

        self._pending.append(setpoint)
        self._node.get_logger().info(
            f"{self.name}: queued setpoint {setpoint} ({len(self._pending)} pending)"
        )

    def update(self) -> py_trees.common.Status:
        if not self._pending:
            self._node.get_logger().info(
                f"{self.name}: waiting for a setpoint on '{self.SETPOINT_TOPIC}'",
                throttle_duration_sec=10.0,
            )
            return py_trees.common.Status.RUNNING

        setpoint = self._pending.popleft()
        self.blackboard.set(blackboard_keys.CURRENT_WAYPOINT, setpoint)
        self._node.get_logger().info(f"{self.name}: next setpoint {setpoint}")
        send_to_ground(self._node, f"ENG: flying to setpoint {setpoint}")
        return py_trees.common.Status.SUCCESS
