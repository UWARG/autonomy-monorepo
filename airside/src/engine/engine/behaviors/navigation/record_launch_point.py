from __future__ import annotations

import py_trees
import rclpy.node
from engine import blackboard_keys
from engine.constants import RETURN_ALTITUDE_M
from engine.ground_log import send_to_ground
from mavros_msgs.msg import HomePosition, State
from std_srvs.srv import Trigger
from utils.src.types import Coordinate

_STATE_TOPIC = "mavros/state"
_HOME_TOPIC = "mavros/home_position/home"
_HOME_UPDATE_SERVICE = "mavros/home_position/req_update"

# Re-request the home position if the FCU has not answered within this long.
_HOME_REQUEST_RETRY_S = 5.0


class RecordLaunchPoint(py_trees.behaviour.Behaviour):
    """
    Records where the drone was armed as the mission's landing spot.

    ArduPilot resets its home position to the current location on arming, so
    once the drone is armed this requests a fresh home position from the FCU
    and writes it to ``launch_point`` at ``RETURN_ALTITUDE_M`` relative
    altitude. Reading it back from the FCU (rather than sampling GPS here)
    keeps the true arming spot even if the engine restarts mid-flight.

    Returns RUNNING until armed and a home position has been received after
    the request, then SUCCESS.
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
        self._requested_s: float | None = None

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
        self._home_update_client = self._node.create_client(
            srv_type=Trigger, srv_name=_HOME_UPDATE_SERVICE
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def _home_callback(self, msg: HomePosition) -> None:
        self._latest_home = msg
        self._home_received_s = self._now_s()

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def initialise(self) -> None:
        self._requested_s = None

    def update(self) -> py_trees.common.Status:
        if self._latest_state is None:
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{_STATE_TOPIC}'",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not self._latest_state.armed:
            self._node.get_logger().warning(
                f"{self.name}: waiting for the pilot to arm",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        # Only trust a home position that arrived after arming was seen, since
        # ArduPilot also sets a provisional home at first GPS lock.
        if self._requested_s is not None and self._home_received_s >= self._requested_s:
            return self._record_launch_point()

        if (
            self._requested_s is None
            or self._now_s() - self._requested_s > _HOME_REQUEST_RETRY_S
        ):
            self._request_home()

        return py_trees.common.Status.RUNNING

    def _request_home(self) -> None:
        if not self._home_update_client.service_is_ready():
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{_HOME_UPDATE_SERVICE}' service",
                throttle_duration_sec=5.0,
            )
            return

        self._home_update_client.call_async(Trigger.Request())
        self._requested_s = self._now_s()
        self._node.get_logger().info(f"{self.name}: requesting home position")

    def _record_launch_point(self) -> py_trees.common.Status:
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
