from __future__ import annotations

import py_trees
import rclpy.node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL


class Land(py_trees.behaviour.Behaviour):
    """
    Lands the drone at its current position via the MAVROS land command.

    Sends the land command once, then returns RUNNING until the flight
    controller disarms. Returns FAILURE if the land command is rejected.
    """

    STATE_TOPIC = "mavros/state"
    LAND_SERVICE = "mavros/cmd/land"

    def __init__(self, name: str = "Land") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None
        self._land_future = None
        self._land_accepted = False

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=self.STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )
        self._land_client = self._node.create_client(
            srv_type=CommandTOL, srv_name=self.LAND_SERVICE
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def initialise(self) -> None:
        self._land_future = None
        self._land_accepted = False

    def update(self) -> py_trees.common.Status:
        if self._latest_state is None:
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{self.STATE_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._land_accepted:
            return self._command_land()

        if not self._latest_state.armed:
            self._node.get_logger().info(f"{self.name}: landed and disarmed")
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().info(
            f"{self.name}: descending",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING

    def _command_land(self) -> py_trees.common.Status:
        """Send the land service call and track its response."""

        if self._land_future is None:
            if not self._land_client.service_is_ready():
                self._node.get_logger().warning(
                    f"{self.name}: waiting for '{self.LAND_SERVICE}' service",
                    throttle_duration_sec=5.0,
                )
                return py_trees.common.Status.RUNNING

            self._land_future = self._land_client.call_async(CommandTOL.Request())
            self._node.get_logger().info(f"{self.name}: commanding land")
            return py_trees.common.Status.RUNNING

        if not self._land_future.done():
            return py_trees.common.Status.RUNNING

        response = self._land_future.result()
        self._land_future = None
        if response is None or not response.success:
            self._node.get_logger().error(
                f"{self.name}: land command rejected: {response}"
            )
            return py_trees.common.Status.FAILURE

        self._land_accepted = True
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._land_future = None
            self._land_accepted = False
