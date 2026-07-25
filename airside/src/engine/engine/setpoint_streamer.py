"""High-rate follow control, safety reflexes, and latched command authority."""

from __future__ import annotations

import math
from typing import Optional

import rclpy.node
from airside_interfaces.msg import TrackedTarget
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from follow_authority import (
    AuthorityAction,
    AuthorityConfig,
    AuthorityInputs,
    FollowAuthority,
    StopReason,
)
from follow_runtime import LatestObservationController, RuntimeObservation
from mavros_msgs.msg import PositionTarget, RCIn, State
from mavros_msgs.srv import SetMode
from mavros_setpoint import body_velocity_to_flu, slew
from stack_config import DEPLOYED, StackConfig
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger

SETPOINT_TOPIC = "mavros/setpoint_raw/local"
TARGET_TOPIC = "perception/target"
CANDIDATE_TOPIC = "perception/target_candidate"
DIAGNOSTICS_TOPIC = "follow/diagnostics"
RESET_TOPIC = "perception/reset_target"
ACQUIRE_TOPIC = "perception/acquire_target"


class SetpointStreamer:
    """Compute each command from the newest capture at the stream rate.

    This production component never requests GUIDED, arming, or takeoff. It
    only requests BRAKE/LOITER as terminal safety actions.
    """

    def __init__(self, node: rclpy.node.Node, config: StackConfig = DEPLOYED) -> None:
        self._node = node
        self._config = config

        self._declare("kill_channel", 7)
        self._declare("enable_channel", 8)
        self._declare("rc_high_pwm", 1700)
        self._declare("airborne_altitude_m", 1.0)
        self._declare("props_off_hitl", False)
        self._declare("detector_stride", 1)
        self._declare("max_validated_range_m", 3.0)

        self._kill_channel = int(node.get_parameter("kill_channel").value)
        self._enable_channel = int(node.get_parameter("enable_channel").value)
        self._rc_high_pwm = int(node.get_parameter("rc_high_pwm").value)
        self._airborne_altitude_m = float(
            node.get_parameter("airborne_altitude_m").value
        )
        props_off = bool(node.get_parameter("props_off_hitl").value)
        detector_stride = int(node.get_parameter("detector_stride").value)
        self._max_validated_range_m = float(
            node.get_parameter("max_validated_range_m").value
        )

        self._authority = FollowAuthority(
            AuthorityConfig(
                fc_state_freshness_s=config.fc_state_freshness_s,
                lost_target_timeout_s=config.safety.lost_timeout_s,
                props_off_bypass_airborne=props_off,
            )
        )
        self._controller = LatestObservationController(
            follow=config.follow,
            reflex=config.reflex,
            freshness_s=config.target_freshness_s,
            ema_alpha=config.ema_alpha,
            detector_stride=detector_stride,
        )

        self._state: Optional[State] = None
        self._state_rx_s: Optional[float] = None
        self._altitude_m = 0.0
        self._rc_kill = False
        self._streamed = (0.0, 0.0, 0.0)
        self._candidate_capture_s: Optional[float] = None
        self._last_mode_request = ""
        self._last_mode_request_s = 0.0
        self._last_clear_reason: Optional[StopReason] = None
        self._last_authority_report = None
        self._out_of_range_active = False
        self._latest_track_id = -1

        self._setpoint_pub = node.create_publisher(PositionTarget, SETPOINT_TOPIC, 10)
        self._diagnostics_pub = node.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, 10
        )
        self._reset_pub = node.create_publisher(Empty, RESET_TOPIC, 10)
        self._acquire_pub = node.create_publisher(Empty, ACQUIRE_TOPIC, 10)
        node.create_subscription(TrackedTarget, TARGET_TOPIC, self._on_target, 10)
        node.create_subscription(TrackedTarget, CANDIDATE_TOPIC, self._on_candidate, 10)
        node.create_subscription(State, "mavros/state", self._on_state, 10)
        node.create_subscription(RCIn, "mavros/rc/in", self._on_rc, 10)
        # Relative altitude is sufficient to enforce the enable precondition.
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data

        node.create_subscription(
            PoseStamped,
            "mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )
        self._set_mode = node.create_client(SetMode, "mavros/set_mode")
        node.create_service(SetBool, "follow/set_enabled", self._set_enabled)
        node.create_service(Trigger, "follow/reset_target", self._reset_target)
        node.create_timer(1.0 / config.stream_hz, self._tick)
        node.create_timer(0.5, self._publish_diagnostics)
        node.get_logger().info(
            f"follow streamer ready: {config.stream_hz:.0f} Hz, EMA alpha={config.ema_alpha:.1f}, "
            f"kill=CH{self._kill_channel}, enable=CH{self._enable_channel}, "
            f"props_off_hitl={props_off}; pilot retains arm/takeoff/mode authority"
        )

    def _declare(self, name: str, default) -> None:
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default)

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_target(self, msg: TrackedTarget) -> None:
        capture_s = self._stamp_s(msg.header.stamp)
        if capture_s <= 0.0:
            return
        detector_capture_s = self._stamp_s(msg.detector_stamp)
        detector_confirmed = bool(msg.detector_confirmed)
        self._latest_track_id = int(msg.track_id)
        position = msg.position
        range_m = math.sqrt(
            float(position.x) ** 2 + float(position.y) ** 2 + float(position.z) ** 2
        )
        out_of_range = detector_confirmed and (
            not bool(msg.within_validated_range)
            or range_m > self._max_validated_range_m
        )
        if out_of_range:
            self._out_of_range_active = True
        elif detector_confirmed:
            self._out_of_range_active = False
        self._controller.update(
            RuntimeObservation(
                x_m=float(position.x),
                y_m=float(position.y),
                z_m=float(position.z),
                track_id=int(msg.track_id),
                sequence_num=int(msg.sequence_num),
                capture_time_s=capture_s,
                receive_time_s=self._now_s(),
                detector_capture_time_s=(
                    detector_capture_s if detector_capture_s > 0.0 else capture_s
                ),
                detector_sequence_num=int(msg.detector_sequence_num),
                detector_confirmed=detector_confirmed,
                spatial_control_valid=not out_of_range,
            )
        )

    def _on_candidate(self, msg: TrackedTarget) -> None:
        capture_s = self._stamp_s(msg.header.stamp)
        detector_capture_s = self._stamp_s(msg.detector_stamp)
        position = msg.position
        range_m = math.sqrt(position.x**2 + position.y**2 + position.z**2)
        if (
            capture_s > 0.0
            and bool(msg.detector_confirmed)
            and bool(msg.within_validated_range)
            and range_m <= self._max_validated_range_m
            and position.z > 0.0
            and all(
                math.isfinite(value) for value in (position.x, position.y, position.z)
            )
        ):
            self._candidate_capture_s = (
                detector_capture_s if detector_capture_s > 0.0 else capture_s
            )

    def _on_state(self, msg: State) -> None:
        self._state = msg
        self._state_rx_s = self._now_s()

    def _on_pose(self, msg) -> None:
        self._altitude_m = float(msg.pose.position.z)

    def _channel_high(self, msg: RCIn, channel: int) -> bool:
        index = channel - 1
        return (
            0 <= index < len(msg.channels) and msg.channels[index] >= self._rc_high_pwm
        )

    def _on_rc(self, msg: RCIn) -> None:
        self._rc_kill = self._channel_high(msg, self._kill_channel)
        enable_high = self._channel_high(msg, self._enable_channel)
        accepted = self._authority.update_rc_enable(
            enable_high,
            self._inputs(target_valid=self._target_available_for_enable()),
        )
        if accepted:
            self._last_clear_reason = None
            self._acquire_pub.publish(Empty())
            self._node.get_logger().info("follow enabled by RC rising edge")

    def _target_available_for_enable(self) -> bool:
        output = self._controller.evaluate(self._now_s())
        candidate_fresh = (
            self._candidate_capture_s is not None
            and 0.0
            <= self._now_s() - self._candidate_capture_s
            <= self._config.target_freshness_s
        )
        return output.fresh or candidate_fresh

    def _inputs(self, proximity: bool = False, target_valid: Optional[bool] = None):
        now = self._now_s()
        output = self._controller.evaluate(now)
        state = self._state
        return AuthorityInputs(
            now_s=now,
            fc_state_rx_s=self._state_rx_s,
            connected=bool(state and state.connected),
            mode=state.mode if state else "",
            armed=bool(state and state.armed),
            airborne=self._altitude_m >= self._airborne_altitude_m,
            target_valid=output.fresh if target_valid is None else target_valid,
            target_out_of_range=self._out_of_range_active,
            proximity_emergency=proximity,
            rc_kill=self._rc_kill,
        )

    def _set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            response.success = self._authority.request_enable(
                self._inputs(target_valid=self._target_available_for_enable())
            )
            response.message = (
                "follow enabled"
                if response.success
                else "enable rejected: require fresh FC state, "
                "GUIDED, armed/airborne (unless props-off HITL), valid target, and kill low"
            )
            if response.success:
                self._last_clear_reason = None
                self._acquire_pub.publish(Empty())
        else:
            self._authority.disable()
            self._clear_target(StopReason.EXPLICIT_DISABLE)
            response.success = True
            response.message = "follow disabled and target lock cleared"
        return response

    def _reset_target(self, _request: Trigger.Request, response: Trigger.Response):
        self._authority.reset_target()
        self._controller.clear()
        self._clear_target(StopReason.RESET_TARGET)
        response.success = True
        response.message = (
            "follow disabled; target lock cleared; new enable edge required"
        )
        return response

    def _clear_target(self, reason: StopReason) -> None:
        if self._last_clear_reason is reason:
            return
        self._controller.clear()
        self._candidate_capture_s = None
        self._out_of_range_active = False
        self._latest_track_id = -1
        self._reset_pub.publish(Empty())
        self._last_clear_reason = reason

    def _publish_velocity(self, command) -> None:
        v_forward, v_right, v_down, yaw_rate = command
        flu = body_velocity_to_flu(v_forward, v_right, v_down, yaw_rate)
        msg = PositionTarget()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = flu.linear_x
        msg.velocity.y = flu.linear_y
        msg.velocity.z = flu.linear_z
        msg.yaw_rate = flu.angular_z
        self._setpoint_pub.publish(msg)

    def _publish_smoothed(self, setpoint) -> None:
        target = (setpoint.v_forward, setpoint.v_right, setpoint.v_down)
        delta = self._config.cmd_slew_mps2 / self._config.stream_hz
        self._streamed = tuple(
            slew(previous, desired, delta)
            for previous, desired in zip(self._streamed, target)
        )
        self._publish_velocity((*self._streamed, setpoint.yaw_rate))

    def _request_mode(self, mode: str) -> None:
        now = self._now_s()
        if mode == self._last_mode_request and now - self._last_mode_request_s < 1.0:
            return
        if not self._set_mode.service_is_ready():
            return
        request = SetMode.Request()
        request.custom_mode = mode
        self._set_mode.call_async(request)
        self._last_mode_request = mode
        self._last_mode_request_s = now
        self._node.get_logger().error(f"terminal follow stop requesting {mode}")

    def _tick(self) -> None:
        now = self._now_s()
        output = self._controller.evaluate(now)
        result = self._authority.step(
            self._inputs(
                proximity=output.proximity_emergency,
                target_valid=output.fresh,
            )
        )
        report = (result.state, result.stop_reason)
        if report != self._last_authority_report:
            self._node.get_logger().warning(
                f"follow authority: state={result.state.value} "
                f"reason={result.stop_reason.value} action={result.action.value}"
            )
            self._last_authority_report = report
            # Emit the reason before the corresponding zero/BRAKE setpoint so
            # the HITL recorder can measure reason-specific stop latency.
            self._publish_diagnostics()
        if result.clear_target_lock:
            self._clear_target(result.stop_reason)

        if result.action is AuthorityAction.STREAM and output.setpoint is not None:
            self._publish_smoothed(output.setpoint)
        elif result.action is AuthorityAction.ZERO:
            self._streamed = (0.0, 0.0, 0.0)
            self._publish_velocity((0.0, 0.0, 0.0, 0.0))
        elif result.action is AuthorityAction.BRAKE:
            self._streamed = (0.0, 0.0, 0.0)
            self._publish_velocity((0.0, 0.0, 0.0, 0.0))
            self._request_mode("BRAKE")
        elif result.action is AuthorityAction.LOITER:
            self._streamed = (0.0, 0.0, 0.0)
            self._request_mode("LOITER")
        else:
            # RELEASE is intentional: pilot mode changes stop all publications
            # no later than this one streamer tick.
            self._streamed = (0.0, 0.0, 0.0)

    def _publish_diagnostics(self) -> None:
        now = self._now_s()
        output = self._controller.evaluate(now)
        metrics = self._controller.metrics()
        latest = self._controller.latest
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self._node.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "follow/authority"
        status.hardware_id = "follow_stack"
        status.level = (
            DiagnosticStatus.OK if self._authority.enabled else DiagnosticStatus.WARN
        )
        status.message = self._authority.stop_reason.value
        values = {
            "authority_state": self._authority.state.value,
            "enabled": str(self._authority.enabled).lower(),
            "target_age_s": f"{output.target_age_s:.6f}",
            "effective_fps": f"{metrics.effective_fps:.3f}",
            "detector_fps": f"{metrics.detector_fps:.3f}",
            "tracker_fps": f"{metrics.tracker_fps:.3f}",
            "latency_p50_ms": f"{metrics.latency_p50_s * 1000.0:.3f}",
            "latency_p95_ms": f"{metrics.latency_p95_s * 1000.0:.3f}",
            "latency_p99_ms": f"{metrics.latency_p99_s * 1000.0:.3f}",
            "sequence_gaps": str(metrics.sequence_gaps),
            "detector_sequence_gaps": str(metrics.detector_sequence_gaps),
            "tracker_sequence_gaps": str(metrics.tracker_sequence_gaps),
            "tracker_only_frames": str(metrics.tracker_only_frames),
            "lock_id": str(self._latest_track_id if latest else -1),
            "target_condition": (
                StopReason.OUT_OF_VALIDATED_RANGE.value
                if self._out_of_range_active
                else "in_range"
            ),
            "max_validated_range_m": f"{self._max_validated_range_m:.3f}",
            "stop_reason": self._authority.stop_reason.value,
        }
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        diagnostic.status = [status]
        self._diagnostics_pub.publish(diagnostic)
