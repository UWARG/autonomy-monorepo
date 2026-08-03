"""
Requests MAVLink telemetry streams from the flight controller.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from engine.constants import BASELINE_STREAM_RATE_HZ, STREAM_RATE_REQUESTS_HZ
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval, StreamRate


class ConfigureStreamRates(py_trees.behaviour.Behaviour):
    """
    Configures FCU telemetry stream rates once MAVROS is connected.

    Waits for the MAVROS <-> FCU link, enables all legacy streams at a
    baseline rate, then requests the per-message rates in
    ``STREAM_RATE_REQUESTS_HZ`` via ``SET_MESSAGE_INTERVAL``. Returns
    RUNNING until every request has been answered, then SUCCESS.
    Rejected per-message requests only warn since the baseline streams
    still cover them.
    """

    STATE_TOPIC = "mavros/state"
    STREAM_RATE_SERVICE = "mavros/set_stream_rate"
    MESSAGE_INTERVAL_SERVICE = "mavros/set_message_interval"

    def __init__(self, name: str = "ConfigureStreamRates") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._latest_state: State | None = None
        self._stream_rate_future = None
        self._interval_futures: dict[int, object] = {}

        self._state_sub = self._node.create_subscription(
            msg_type=State,
            topic=self.STATE_TOPIC,
            callback=self._state_callback,
            qos_profile=10,
        )
        self._stream_rate_client = self._node.create_client(
            srv_type=StreamRate, srv_name=self.STREAM_RATE_SERVICE
        )
        self._interval_client = self._node.create_client(
            srv_type=MessageInterval, srv_name=self.MESSAGE_INTERVAL_SERVICE
        )

    def _state_callback(self, msg: State) -> None:
        self._latest_state = msg

    def initialise(self) -> None:
        self._stream_rate_future = None
        self._interval_futures = {}

    def update(self) -> py_trees.common.Status:
        if self._latest_state is None or not self._latest_state.connected:
            self._node.get_logger().warning(
                f"{self.name}: waiting for MAVROS to connect to the FCU",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if not (
            self._stream_rate_client.service_is_ready()
            and self._interval_client.service_is_ready()
        ):
            self._node.get_logger().warning(
                f"{self.name}: waiting for MAVROS stream rate services",
                throttle_duration_sec=5.0,
            )
            return py_trees.common.Status.RUNNING

        if self._stream_rate_future is None:
            request = StreamRate.Request()
            request.stream_id = StreamRate.Request.STREAM_ALL
            request.message_rate = BASELINE_STREAM_RATE_HZ
            request.on_off = True
            self._stream_rate_future = self._stream_rate_client.call_async(request)

            for message_id, rate_hz in STREAM_RATE_REQUESTS_HZ.items():
                interval_request = MessageInterval.Request()
                interval_request.message_id = message_id
                interval_request.message_rate = rate_hz
                self._interval_futures[message_id] = self._interval_client.call_async(
                    interval_request
                )

            self._node.get_logger().info(
                f"{self.name}: requested all streams at {BASELINE_STREAM_RATE_HZ}Hz "
                f"and message intervals {STREAM_RATE_REQUESTS_HZ}"
            )
            return py_trees.common.Status.RUNNING

        pending = [
            future
            for future in [self._stream_rate_future, *self._interval_futures.values()]
            if not future.done()
        ]
        if pending:
            return py_trees.common.Status.RUNNING

        for message_id, future in self._interval_futures.items():
            response = future.result()
            if response is None or not response.success:
                self._node.get_logger().warning(
                    f"{self.name}: SET_MESSAGE_INTERVAL for message ID "
                    f"{message_id} rejected; relying on baseline streams"
                )

        self._node.get_logger().info(f"{self.name}: stream rates configured")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status != py_trees.common.Status.SUCCESS:
            self._stream_rate_future = None
            self._interval_futures = {}
