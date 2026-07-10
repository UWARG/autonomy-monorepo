from __future__ import annotations

import py_trees
import rclpy.node
from engine.constants import (
    TAKEOFF_AIRBORNE_THRESHOLD_M,
    TAKEOFF_ALTITUDE_M,
    TAKEOFF_ALTITUDE_TOLERANCE_M,
)
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class Takeoff(py_trees.behaviour.Behaviour):
    """
    Commands a guided takeoff to ``TAKEOFF_ALTITUDE_M``.

    Waits for the pilot to arm the drone and select GUIDED mode,
    then sends a MAVROS takeoff command. Returns RUNNING until
    the relative altitude is within ``TAKEOFF_ALTITUDE_TOLERANCE_M``
    of the target, then SUCCESS. Returns FAILURE if the takeoff
    command is rejected.

    If the drone is already flying (armed and above
    ``TAKEOFF_AIRBORNE_THRESHOLD_M``), succeeds immediately without
    commanding a takeoff.
    """

    STATE_TOPIC = "mavros/state"
    REL_ALT_TOPIC = "mavros/global_position/rel_alt"
    TAKEOFF_SERVICE = "mavros/cmd/takeoff"

    GUIDED_MODE = "GUIDED"

    def __init__(self, name: str = "Takeoff") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None
        self._latest_rel_alt_m: float | None = None
        self._takeoff_future = None
        self._takeoff_accepted = False

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=self.STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )
        self._rel_alt_sub = self._node.create_subscription(
            msg_type=Float64,
            topic=self.REL_ALT_TOPIC,
            callback=self._rel_alt_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._takeoff_client = self._node.create_client(
            srv_type=CommandTOL, srv_name=self.TAKEOFF_SERVICE
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
                f"{self.name}: waiting for '{self.STATE_TOPIC}' and "
                f"'{self.REL_ALT_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if (
            not self._takeoff_accepted
            and self._latest_state.armed
            and self._latest_rel_alt_m >= TAKEOFF_AIRBORNE_THRESHOLD_M
        ):
            self._node.get_logger().info(
                f"{self.name}: already flying at {self._latest_rel_alt_m:.1f}m, "
                "skipping takeoff"
            )
            return py_trees.common.Status.SUCCESS

        if self._latest_state.mode != self.GUIDED_MODE:
            self._node.get_logger().warning(
                f"{self.name}: flight controller in '{self._latest_state.mode}' "
                f"mode, waiting for the pilot to select '{self.GUIDED_MODE}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._latest_state.armed:
            self._node.get_logger().warning(
                f"{self.name}: waiting for the pilot to arm",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._takeoff_accepted:
            return self._command_takeoff()

        if (
            self._latest_rel_alt_m
            >= TAKEOFF_ALTITUDE_M - TAKEOFF_ALTITUDE_TOLERANCE_M
        ):
            self._node.get_logger().info(
                f"{self.name}: reached {self._latest_rel_alt_m:.1f}m"
            )
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: climbing, {self._latest_rel_alt_m:.1f}m / "
            f"{TAKEOFF_ALTITUDE_M:.1f}m",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING

    def _command_takeoff(self) -> py_trees.common.Status:
        """Send the takeoff service call and track its response."""

        if self._takeoff_future is None:
            if not self._takeoff_client.service_is_ready():
                self._node.get_logger().warning(
                    f"{self.name}: waiting for '{self.TAKEOFF_SERVICE}' service",
                    throttle_duration_sec=5.0,
                )
                return py_trees.common.Status.RUNNING

            request = CommandTOL.Request()
            request.altitude = TAKEOFF_ALTITUDE_M
            self._takeoff_future = self._takeoff_client.call_async(request)
            self._node.get_logger().info(
                f"{self.name}: commanding takeoff to {TAKEOFF_ALTITUDE_M:.1f}m"
            )
            return py_trees.common.Status.RUNNING

        if not self._takeoff_future.done():
            return py_trees.common.Status.RUNNING

        response = self._takeoff_future.result()
        self._takeoff_future = None
        if response is None or not response.success:
            self._node.get_logger().error(
                f"{self.name}: takeoff command rejected: {response}"
            )
            return py_trees.common.Status.FAILURE

        self._takeoff_accepted = True
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._takeoff_future = None
            self._takeoff_accepted = False
