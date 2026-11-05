import math
from functools import partial
from time import monotonic

import diagnostic_updater
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from mavros_msgs.msg import State, SysStatus
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Empty, Float64

from airside_interfaces.srv import CapturePhoto
from engine.preflight import (
    ERROR,
    OK,
    STALE,
    Check,
    Stream,
    image_check,
    overall,
    prearm_check,
)


class PreflightNode(Node):
    def __init__(self):
        super().__init__("preflight_node")
        defaults = {
            "groundstation_timeout_s": 3.0,
            "camera_timeout_s": 2.0,
            "fcu_timeout_s": 3.0,
            "camera_min_mean": 5.0,
            "camera_min_std": 2.0,
            "topic_timeout_s": 3.0,
        }
        self._settings = {
            key: self.declare_parameter(key, value).value
            for key, value in defaults.items()
        }
        for key in ("camera_min_mean", "camera_min_std"):
            if (
                not math.isfinite(self._settings[key])
                or not 0 <= self._settings[key] <= 255
            ):
                raise ValueError(f"{key} must be finite and between 0 and 255")
        self._groundstation = Stream(self._settings["groundstation_timeout_s"])
        self._camera = Stream(self._settings["camera_timeout_s"])
        self._state_stream = Stream(self._settings["fcu_timeout_s"])
        self._status_stream = Stream(self._settings["fcu_timeout_s"])
        self._state = None
        self._status = None
        self._image_result = Check(STALE, "No image inspected")
        self._last_image_inspection = -math.inf
        self._image_stamp = None
        self._subscriptions = [
            self.create_subscription(
                Empty,
                "/groundstation/heartbeat",
                self._on_groundstation,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, "/camera/image_raw", self._on_image, qos_profile_sensor_data
            ),
            self.create_subscription(
                State, "/mavros/state", self._on_state, qos_profile_sensor_data
            ),
            self.create_subscription(
                SysStatus,
                "/mavros/sys_status",
                self._on_status,
                qos_profile_sensor_data,
            ),
        ]
        # These streams run continuously. Mission commands can be silent before flight.
        topics = [
            ("/heartbeat", Empty, 0.5),
            ("/mavros/global_position/global", NavSatFix, 1.0),
            ("/mavros/global_position/rel_alt", Float64, 1.0),
        ]
        self._topics = {}
        for topic, msg_type, min_hz in topics:
            stream = Stream(self._settings["topic_timeout_s"], min_hz)
            self._topics[topic] = stream
            self._subscriptions.append(
                self.create_subscription(
                    msg_type,
                    topic,
                    partial(self._on_topic, stream),
                    qos_profile_sensor_data,
                )
            )
        self._capture = self.create_client(CapturePhoto, "/camera/capture")
        self._results = self._checks()
        self._updater = diagnostic_updater.Updater(
            self, starting_up_status=DiagnosticStatus.STALE
        )
        self._updater.setHardwareID("3ds-companion")
        for name in self._results:
            self._updater.add(name, partial(self._report, name))
        self._updater.add("readiness", self._report_readiness)
        # A paused simulation clock must not leave a previously green status latched.
        self._updater.timer.cancel()
        self._timer = self.create_timer(
            1.0,
            self._publish,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _on_groundstation(self, msg):
        self._groundstation.tick(monotonic())

    def _on_topic(self, stream, msg):
        stream.tick(monotonic())

    def _on_state(self, msg):
        self._state_stream.tick(monotonic())
        if not msg.connected:
            self._status = None
            self._status_stream.times.clear()
        self._state = msg

    def _on_status(self, msg):
        # Do not retain status received before a known live connection.
        if self._state is not None and self._state.connected:
            self._status = msg
            self._status_stream.tick(monotonic())

    def _on_image(self, msg):
        now = monotonic()
        self._camera.tick(now)
        if now - self._last_image_inspection < 0.5:
            return
        self._last_image_inspection = now
        self._image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._image_result = image_check(
            msg.data,
            msg.width,
            msg.height,
            msg.step,
            msg.encoding,
            self._settings["camera_min_mean"],
            self._settings["camera_min_std"],
        )

    def _checks(self):
        now = monotonic()
        camera = self._camera.check(now)
        if camera.level == OK:
            camera = self._image_result
            ros_now = self.get_clock().now().nanoseconds / 1e9
            if self._image_stamp is None or self._image_stamp <= 0:
                camera = Check(STALE, "Image capture timestamp missing")
            elif (
                not -0.5
                <= ros_now - self._image_stamp
                <= self._settings["camera_timeout_s"]
            ):
                camera = Check(STALE, "Image capture timestamp stale or in future")
        connection = self._state_stream.check(now)
        if connection.level == OK and not self._state.connected:
            connection = Check(ERROR, "MAVROS disconnected from ArduPilot")
        prearm = self._status_stream.check(now)
        if connection.level != OK:
            prearm = Check(STALE, "Waiting for live FCU connection")
        elif prearm.level == OK:
            prearm = prearm_check(
                self._status.sensors_present,
                self._status.sensors_enabled,
                self._status.sensors_health,
                self._state.armed,
            )
        checks = {
            "groundstation": self._groundstation.check(now),
            "camera": camera,
            "fcu_connection": connection,
            "ardupilot_prearm": prearm,
            "photo_service": Check(OK, "Capture service available")
            if self._capture.service_is_ready()
            else Check(ERROR, "Camera capture service unavailable"),
        }
        checks.update(
            {
                f"topic {name}": stream.check(now)
                for name, stream in self._topics.items()
            }
        )
        return checks

    def _report(self, name, stat):
        check = self._results[name]
        stat.summary(check.level, check.message)
        return stat

    def _report_readiness(self, stat):
        checks = self._results
        summary = overall(checks)
        stat.summary(summary.level, summary.message)
        stat.add("ready", str(summary.level == OK).lower())
        for name, check in checks.items():
            stat.add(name, check.message)
        return stat

    def _publish(self):
        self._results = self._checks()
        self._updater.force_update()


def main(args=None):
    rclpy.init(args=args)
    node = PreflightNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
