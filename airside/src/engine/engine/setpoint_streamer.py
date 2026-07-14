from __future__ import annotations

import math
from typing import Optional

import py_trees
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from flight_modes import ControlAction, FlightInputs, decide, should_request
from geometry_msgs.msg import PointStamped, PoseStamped
from handoff import HandoffAction, HandoffSequencer
from hold_policy import HoldAction, HoldPolicy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_setpoint import body_velocity_to_flu, slew
from range_rate import ReflexMonitor
from stack_config import DEPLOYED, StackConfig
from sensor_msgs.msg import NavSatFix

from engine.behaviors.read_target import (
    KEY_COMMAND,
    KEY_COMMAND_STAMP,
    KEY_ESTOP_EMERGENCY,
    KEY_ESTOP_HOLD,
    KEY_ESTOP_RECEDE,
)

SETPOINT_TOPIC = "mavros/setpoint_raw/local"
HOLD_TOPIC = "mavros/setpoint_position/local"
TARGET_TOPIC = "perception/target"


class SetpointStreamer:
    def __init__(
        self,
        node: rclpy.node.Node,
        config: StackConfig = DEPLOYED,
        auto_arm: bool = False,
    ) -> None:
        self._node = node
        self._config = config
        self._auto_arm = auto_arm

        self._hold = HoldPolicy(config.hold)
        self._reflex = ReflexMonitor(config.reflex)
        self._handoff = HandoffSequencer(config.handoff)
        if not auto_arm:
            
            self._handoff.mark_complete()

        self._state: Optional[State] = None
        self._gps_fix = False  
        self._gps_rx_s = 0.0
        self._pose: Optional[PoseStamped] = None
        self._pose_rx_s: Optional[float] = None
        self._alt = 0.0 
        self._hold_pose: Optional[PoseStamped] = None
        self._target_range: Optional[float] = None
        self._target_rx_s = 0.0

        self._last_mode_req = ""
        self._last_mode_req_s = 0.0
        self._last_action: Optional[ControlAction] = None
        self._streamed = (0.0, 0.0, 0.0)

        self.blackboard = py_trees.blackboard.Client(name="SetpointStreamer")
        for key in (
            KEY_COMMAND,
            KEY_COMMAND_STAMP,
            KEY_ESTOP_EMERGENCY,
            KEY_ESTOP_RECEDE,
            KEY_ESTOP_HOLD,
        ):
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

        self._pub = node.create_publisher(PositionTarget, SETPOINT_TOPIC, 10)
        self._pos_pub = node.create_publisher(PoseStamped, HOLD_TOPIC, 10)
        node.create_subscription(PointStamped, TARGET_TOPIC, self._on_target, 10)
        node.create_subscription(State, "mavros/state", self._on_state, 10)
        node.create_subscription(
            NavSatFix, "mavros/global_position/global", self._on_global, qos_profile_sensor_data
        )
        node.create_subscription(
            PoseStamped, "mavros/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self._set_mode = node.create_client(SetMode, "mavros/set_mode")
        self._arming = node.create_client(CommandBool, "mavros/cmd/arming")
        self._takeoff = node.create_client(CommandTOL, "mavros/cmd/takeoff")
        node.create_timer(1.0 / config.stream_hz, self._tick)
        node.get_logger().info(
            f"SetpointStreamer: {config.stream_hz} Hz (auto_arm={auto_arm}, "
            f"hard_min={config.reflex.hard_min_m} m, a_brake={config.reflex.a_brake} m/s^2, "
            f"command_stale_s={config.command_stale_s})"
        )

    # --- subscriptions ---
    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_global(self, msg: NavSatFix) -> None:
        self._gps_fix = msg.status.status >= NavSatFix().status.STATUS_FIX
        self._gps_rx_s = self._now_s()

    def _ekf_ok(self) -> bool:
        return self._gps_fix and (self._now_s() - self._gps_rx_s) < 1.0

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg
        self._pose_rx_s = self._now_s()
        self._alt = msg.pose.position.z

    def _on_target(self, msg: PointStamped) -> None:
        p = msg.point
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)) or p.z <= 0.0:
            return
        self._target_range = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
        self._target_rx_s = self._now_s()

    # --- helpers ---
    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _pose_age_s(self, now: float) -> float:
        if self._pose_rx_s is None:
            return math.inf
        return now - self._pose_rx_s

    def _reflex_danger(self, now: float) -> bool:
        if (
            self._target_range is None
            or (now - self._target_rx_s) > self._config.target_freshness_s
        ):
            return self._reflex.update(None, now)
        return self._reflex.update(self._target_range, self._target_rx_s)

    def _publish_velocity(self, v_forward, v_right, v_down, yaw_rate) -> None:
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
        msg.velocity.x = flu.linear_x  # forward
        msg.velocity.y = flu.linear_y  # left
        msg.velocity.z = flu.linear_z  # up
        msg.yaw_rate = flu.angular_z  # CCW (ENU)
        self._pub.publish(msg)

    def _publish_hold_pose(self) -> None:
        # Republish the latched ENU pose on the position interface
        if self._hold_pose is None:
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
            return
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._hold_pose.header.frame_id
        msg.pose = self._hold_pose.pose
        self._pos_pub.publish(msg)

    def _request_mode(self, mode: str) -> None:
        state_mode = self._state.mode if self._state else ""
        now = self._now_s()
        if not should_request(state_mode, mode, self._last_mode_req, self._last_mode_req_s, now):
            return
        if self._set_mode.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self._set_mode.call_async(req)
            self._last_mode_req = mode
            self._last_mode_req_s = now
            self._node.get_logger().warn(f"SetpointStreamer: requesting {mode} (from {state_mode})")

    def _publish_follow(self, cmd, now: float) -> None:
        v_forward, v_right, v_down, yaw_rate = cmd
        speed = max(abs(v_forward), abs(v_right), abs(v_down))
        dv = self._config.cmd_slew_mps2 / self._config.stream_hz
        v_forward = slew(self._streamed[0], v_forward, dv)
        v_right = slew(self._streamed[1], v_right, dv)
        v_down = slew(self._streamed[2], v_down, dv)
        self._streamed = (v_forward, v_right, v_down)
        status = self._hold.step(
            speed_mps=speed,
            yaw_rate=yaw_rate,
            pose_age_s=self._pose_age_s(now),
            ekf_ok=self._ekf_ok(),
            now_s=now,
        )
        if status.action is HoldAction.LATCH:
            self._hold_pose = self._pose 
            p = self._hold_pose.pose.position
            self._node.get_logger().info(
                f"SetpointStreamer: position hold engaged at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) "
                f"[latch #{status.relatch_count}]"
            )
        elif status.action is HoldAction.FOLLOW and self._hold_pose is not None:
            self._node.get_logger().info(f"SetpointStreamer: hold released ({status.reason})")
            self._hold_pose = None
        if status.action in (HoldAction.LATCH, HoldAction.HOLD):
            self._publish_hold_pose()
        else:
            self._publish_velocity(v_forward, v_right, v_down, yaw_rate)

    def _log_transition(self, action: ControlAction) -> None:
        if action is not self._last_action:
            self._node.get_logger().info(
                f"SetpointStreamer: action {self._last_action.value if self._last_action else 'none'}"
                f" -> {action.value}"
            )
            self._last_action = action

    # --- main loop ---
    def _tick(self) -> None:
        now = self._now_s()
        if not self._handoff.is_complete:
            self._run_handoff(now)
            return

        cmd = self.blackboard.get(KEY_COMMAND)
        stamp = self.blackboard.get(KEY_COMMAND_STAMP) or 0.0
        inputs = FlightInputs(
            in_guided=bool(self._state and self._state.mode == "GUIDED"),
            ekf_ok=self._ekf_ok(),
            command_fresh=cmd is not None and (now - stamp) <= self._config.command_stale_s,
            reflex_danger=self._reflex_danger(now),
            estop_emergency=bool(self.blackboard.get(KEY_ESTOP_EMERGENCY)),
            estop_recede=bool(self.blackboard.get(KEY_ESTOP_RECEDE)),
            estop_hold=bool(self.blackboard.get(KEY_ESTOP_HOLD)),
            holding=self._hold.is_holding,
            armed=bool(self._state and self._state.armed),
        )
        action = decide(inputs)
        self._log_transition(action)

        if action is ControlAction.STREAM_VELOCITY:
            self._publish_follow(cmd, now)
            return
        if action is ControlAction.HOLD_POSITION:
            self._publish_hold_pose()
            return
        self._hold.reset()
        self._hold_pose = None
        self._streamed = (0.0, 0.0, 0.0)
        if action is ControlAction.STREAM_ZERO:
            self._publish_velocity(0.0, 0.0, 0.0, 0.0)
        elif action is ControlAction.SET_BRAKE:
            self._publish_velocity(0.0, 0.0, 0.0, 0.0) 
            self._request_mode("BRAKE")
        elif action is ControlAction.SET_LOITER:
            self._request_mode("LOITER")
        elif action is ControlAction.RELEASE:
            pass

    def _run_handoff(self, now: float) -> None:
        connected = bool(self._state and self._state.connected)
        mode = self._state.mode if self._state else ""
        armed = bool(self._state and self._state.armed)
        action = self._handoff.step(connected, mode, armed, self._alt, now)

        if action is HandoffAction.REQUEST_GUIDED:
            self._request_mode("GUIDED")
        elif action is HandoffAction.REQUEST_ARM:
            if self._arming.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self._arming.call_async(req)
                self._node.get_logger().warn("SetpointStreamer: auto-arm requested (SITL)")
        elif action is HandoffAction.REQUEST_TAKEOFF:
            if self._takeoff.service_is_ready():
                req = CommandTOL.Request()
                req.altitude = float(self._config.handoff.takeoff_alt_m)
                self._takeoff.call_async(req)
                self._handoff.notify_takeoff_sent(now)
                self._node.get_logger().warn(
                    f"SetpointStreamer: takeoff to {self._config.handoff.takeoff_alt_m} m "
                    "requested (SITL)"
                )
        elif action is HandoffAction.COMPLETE:
            self._node.get_logger().info("SetpointStreamer: airborne + GUIDED; follow active.")
