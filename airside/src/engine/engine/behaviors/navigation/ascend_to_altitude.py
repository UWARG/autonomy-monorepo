from __future__ import annotations

import py_trees
import rclpy.node
from engine.constants import (
    DROP_ASCEND_ALTITUDE_M,
    GUIDED_MODE,
    TAKEOFF_ALTITUDE_TOLERANCE_M,
)
from engine.ground_log import send_to_ground
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


_STATE_TOPIC = "mavros/state"
_REL_ALT_TOPIC = "mavros/global_position/rel_alt"
_TAKEOFF_SERVICE = "mavros/cmd/takeoff"


class AscendToAltitude(py_trees.behaviour.Behaviour):
    """
    Climbs back to ``DROP_ASCEND_ALTITUDE_M`` after a payload drop.

    Because the LAND-mode descent disarms the drone on the pad, ascending is a
    fresh takeoff. This waits for the pilot to re-arm and select GUIDED (the
    same human-in-the-loop arming the ``Takeoff`` behavior relies on), then
    commands a MAVROS takeoff to ``DROP_ASCEND_ALTITUDE_M``. Returns RUNNING
    until the relative altitude is within ``TAKEOFF_ALTITUDE_TOLERANCE_M`` of
    the target, then SUCCESS. Returns FAILURE if the takeoff command is
    rejected.
    """

    def __init__(self, name: str = "AscendToAltitude") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None
        self._latest_rel_alt_m: float | None = None
        self._takeoff_future = None
        self._takeoff_accepted = False

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=_STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )
        self._rel_alt_sub = self._node.create_subscription(
            msg_type=Float64,
            topic=_REL_ALT_TOPIC,
            callback=self._rel_alt_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._takeoff_client = self._node.create_client(
            srv_type=CommandTOL, srv_name=_TAKEOFF_SERVICE
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._latest_rel_alt_m = msg.data

    def initialise(self) -> None:
        self._takeoff_future = None
        self._takeoff_accepted = False

    def update(self) -> py_trees.common.Status:
        if self._latest_state is None or self._latest_rel_alt_m is None:
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{_STATE_TOPIC}' and '{_REL_ALT_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if self._latest_state.mode != GUIDED_MODE:
            self._node.get_logger().warning(
                f"{self.name}: flight controller in '{self._latest_state.mode}' "
                f"mode, waiting for the pilot to select '{GUIDED_MODE}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._latest_state.armed:
            self._node.get_logger().warning(
                f"{self.name}: waiting for the pilot to re-arm",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._takeoff_accepted:
            return self._command_takeoff()

        if (
            self._latest_rel_alt_m
            >= DROP_ASCEND_ALTITUDE_M - TAKEOFF_ALTITUDE_TOLERANCE_M
        ):
            self._node.get_logger().info(
                f"{self.name}: reached {self._latest_rel_alt_m:.1f}m"
            )
            send_to_ground(
                self._node, f"ENG: back up at {self._latest_rel_alt_m:.1f}m"
            )
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: climbing, {self._latest_rel_alt_m:.1f}m / "
            f"{DROP_ASCEND_ALTITUDE_M:.1f}m",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING

    def _command_takeoff(self) -> py_trees.common.Status:
        """Send the takeoff service call and track its response."""

        if self._takeoff_future is None:
            if not self._takeoff_client.service_is_ready():
                self._node.get_logger().warning(
                    f"{self.name}: waiting for '{_TAKEOFF_SERVICE}' service",
                    throttle_duration_sec=5.0,
                )
                return py_trees.common.Status.RUNNING

            request = CommandTOL.Request()
            request.altitude = DROP_ASCEND_ALTITUDE_M
            self._takeoff_future = self._takeoff_client.call_async(request)
            self._node.get_logger().info(
                f"{self.name}: commanding ascent to {DROP_ASCEND_ALTITUDE_M:.1f}m"
            )
            send_to_ground(
                self._node, f"ENG: ascending to {DROP_ASCEND_ALTITUDE_M:.0f}m"
            )
            return py_trees.common.Status.RUNNING

        if not self._takeoff_future.done():
            return py_trees.common.Status.RUNNING

        response = self._takeoff_future.result()
        self._takeoff_future = None
        if response is None or not response.success:
            self._node.get_logger().error(
                f"{self.name}: ascent command rejected: {response}"
            )
            send_to_ground(self._node, "ENG: ascent rejected")
            return py_trees.common.Status.FAILURE

        self._takeoff_accepted = True
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._takeoff_future = None
            self._takeoff_accepted = False
