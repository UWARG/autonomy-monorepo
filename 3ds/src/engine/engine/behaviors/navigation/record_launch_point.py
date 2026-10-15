from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import RETURN_ALTITUDE_M
from engine.ground_log import send_to_ground
from mavros_msgs.msg import HomePosition, State
from utils.src.types import Coordinate

_STATE_TOPIC = "mavros/state"
_HOME_TOPIC = "mavros/home_position/home"


class RecordLaunchPoint(py_trees.behaviour.Behaviour):
    """
    Records where the drone was armed as the mission's landing spot.

    ArduPilot resets its home position to the current location on arming, so
    once the drone is seen armed, the next home position streamed by the FCU
    (``HOME_POSITION`` is requested in ``ConfigureStreamRates``) is written to
    ``launch_point`` at ``RETURN_ALTITUDE_M`` relative altitude. Reading it
    back from the FCU (rather than sampling GPS here) keeps the true arming
    spot even if the engine restarts mid-flight.

    Returns RUNNING until armed and a home position has been received after
    arming was seen, then SUCCESS.
    """

    def __init__(self, name: str = "RecordLaunchPoint") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.LAUNCH_POINT, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None
        self._latest_home: HomePosition | None = None
        self._home_received_s = 0.0
        self._armed_seen_s: float | None = None

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=_STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )
        self._home_sub = self._node.create_subscription(
            msg_type=HomePosition,
            topic=_HOME_TOPIC,
            callback=self._home_callback,
            qos_profile=10,
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def _home_callback(self, msg: HomePosition) -> None:
        self._latest_home = msg
        self._home_received_s = self._now_s()

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def initialise(self) -> None:
        self._armed_seen_s = None

    def update(self) -> py_trees.common.Status:
        if self._latest_state is None or not self._latest_state.armed:
            self._armed_seen_s = None
            self._node.get_logger().warning(
                f"{self.name}: waiting for the pilot to arm",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if self._armed_seen_s is None:
            self._armed_seen_s = self._now_s()

        # Only trust a home position that arrived after arming was seen, since
        # ArduPilot also sets a provisional home at first GPS lock.
        if self._home_received_s <= self._armed_seen_s:
            self._node.get_logger().warning(
                f"{self.name}: waiting for a home position on '{_HOME_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        launch_point = Coordinate(
            self._latest_home.geo.latitude,
            self._latest_home.geo.longitude,
            RETURN_ALTITUDE_M,
        )
        self.blackboard.set(blackboard_keys.LAUNCH_POINT, launch_point)
        self._node.get_logger().info(
            f"{self.name}: launch point recorded at {launch_point}"
        )
        send_to_ground(self._node, "ENG: launch point recorded")
        return py_trees.common.Status.SUCCESS
