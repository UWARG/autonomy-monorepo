from __future__ import annotations

import math

import py_trees
import rclpy.node
from airside_interfaces.msg import Coordinate as CoordinateMsg
from airside_interfaces.srv import CapturePhoto
from engine import blackboard_keys
from engine.constants import PHOTO_CAPTURE_TIMEOUT_S
from engine.ground_log import send_to_ground
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


class TakePhoto(py_trees.behaviour.Behaviour):
    """
    Asks the camera node to save a photo tagged with the drone's location.

    Calls the ``camera/capture`` service with ``current_waypoint`` as the
    setpoint and the latest MAVROS fix as the measured position (NaN if no
    fix has arrived). Returns RUNNING until the camera responds, SUCCESS once
    the photo is saved, and FAILURE if the capture is rejected or not
    completed within ``PHOTO_CAPTURE_TIMEOUT_S``.
    """

    CAPTURE_SERVICE = "camera/capture"
    GLOBAL_POSITION_TOPIC = "mavros/global_position/global"
    REL_ALT_TOPIC = "mavros/global_position/rel_alt"

    def __init__(self, name: str = "TakePhoto") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.CURRENT_WAYPOINT, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_fix: NavSatFix | None = None
        self._latest_rel_alt_m: float | None = None
        self._capture_future = None
        self._start_time_s = 0.0

        self._fix_sub = self._node.create_subscription(
            msg_type=NavSatFix,
            topic=self.GLOBAL_POSITION_TOPIC,
            callback=self._fix_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._rel_alt_sub = self._node.create_subscription(
            msg_type=Float64,
            topic=self.REL_ALT_TOPIC,
            callback=self._rel_alt_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self._capture_client = self._node.create_client(
            srv_type=CapturePhoto, srv_name=self.CAPTURE_SERVICE
        )

    def _fix_callback(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _rel_alt_callback(self, msg: Float64) -> None:
        self._latest_rel_alt_m = msg.data

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def initialise(self) -> None:
        self._capture_future = None
        self._start_time_s = self._now_s()

    def update(self) -> py_trees.common.Status:
        if self._now_s() - self._start_time_s > PHOTO_CAPTURE_TIMEOUT_S:
            self._node.get_logger().error(
                f"{self.name}: photo not captured within {PHOTO_CAPTURE_TIMEOUT_S}s"
            )
            send_to_ground(self._node, "ENG: photo capture timed out")
            return py_trees.common.Status.FAILURE

        if self._capture_future is None:
            return self._request_capture()

        if not self._capture_future.done():
            return py_trees.common.Status.RUNNING

        response = self._capture_future.result()
        self._capture_future = None
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            self._node.get_logger().error(f"{self.name}: capture failed: {message}")
            send_to_ground(self._node, "ENG: photo capture failed")
            return py_trees.common.Status.FAILURE

        self._node.get_logger().info(f"{self.name}: saved '{response.path}'")
        send_to_ground(self._node, "ENG: photo captured")
        return py_trees.common.Status.SUCCESS

    def _request_capture(self) -> py_trees.common.Status:
        """Send the capture service call."""

        if not self._capture_client.service_is_ready():
            self._node.get_logger().warning(
                f"{self.name}: waiting for '{self.CAPTURE_SERVICE}' service",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        try:
            setpoint = self.blackboard.get(blackboard_keys.CURRENT_WAYPOINT)
        except KeyError:
            self._node.get_logger().error(f"{self.name}: no current setpoint")
            return py_trees.common.Status.FAILURE

        request = CapturePhoto.Request()
        request.setpoint = CoordinateMsg(
            lat=setpoint.lat, lon=setpoint.lon, alt=setpoint.alt
        )
        request.position = self._measured_position()
        self._capture_future = self._capture_client.call_async(request)
        self._node.get_logger().info(f"{self.name}: capturing at {setpoint}")
        return py_trees.common.Status.RUNNING

    def _measured_position(self) -> CoordinateMsg:
        if self._latest_fix is None or self._latest_rel_alt_m is None:
            return CoordinateMsg(lat=math.nan, lon=math.nan, alt=math.nan)
        return CoordinateMsg(
            lat=self._latest_fix.latitude,
            lon=self._latest_fix.longitude,
            alt=self._latest_rel_alt_m,
        )

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._capture_future = None
