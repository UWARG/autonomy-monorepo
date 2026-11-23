"""
Fence safety behaviors.

``CheckFenceBreach`` is the condition the safety selector branches on,
``WarnNearFence`` logs proximity warnings without touching the mission, and
``RecoverIntoGeofence`` flies the drone to the target the breach check stages.
Warnings are logged and sent to the GCS; the engine never changes flight mode.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from airside_interfaces.msg import Coordinate as CoordinateMsg
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from engine import blackboard_keys
from engine.constants import GUIDED_MODE
from engine.fence import SAFE_HOLD_S, Fence
from engine.ground_log import send_to_ground
from utils.src.types import Coordinate

_STATE_TOPIC = "mavros/state"
_POSITION_TOPIC = "mavros/global_position/global"
_ALTITUDE_TOPIC = "mavros/global_position/rel_alt"
_VELOCITY_TOPIC = "mavros/local_position/velocity_local"
_TARGET_TOPIC = "position_controller/target"

# Position or altitude older than this is treated as unknown; velocity older
# than this is treated as zero.
_TELEMETRY_STALE_S = 2.0


class FenceTelemetry:
    """Mixin that tracks the drone's position, velocity and flight state."""

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

        # Separate client: subclasses use self.blackboard for their own keys.
        self._fence_bb = self.attach_blackboard_client(name=f"{self.name}/fence")
        self._fence_bb.register_key(
            key=blackboard_keys.GEOFENCE, access=py_trees.common.Access.READ
        )

        self._state: State | None = None
        self._fix: NavSatFix | None = None
        self._fix_at_s = 0.0
        self._altitude_m: float | None = None
        self._altitude_at_s = 0.0
        self._velocity: TwistStamped | None = None
        self._velocity_at_s = 0.0

        self._subscriptions = [
            self._node.create_subscription(State, _STATE_TOPIC, self._on_state, 10),
            self._node.create_subscription(
                NavSatFix, _POSITION_TOPIC, self._on_fix, qos_profile_sensor_data
            ),
            self._node.create_subscription(
                Float64, _ALTITUDE_TOPIC, self._on_altitude, qos_profile_sensor_data
            ),
            self._node.create_subscription(
                TwistStamped, _VELOCITY_TOPIC, self._on_velocity, qos_profile_sensor_data
            ),
        ]

    def _fence(self) -> Fence | None:
        try:
            return self._fence_bb.get(blackboard_keys.GEOFENCE)
        except KeyError:
            return None

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_fix(self, msg: NavSatFix) -> None:
        self._fix = msg
        self._fix_at_s = self._now_s()

    def _on_altitude(self, msg: Float64) -> None:
        self._altitude_m = msg.data
        self._altitude_at_s = self._now_s()

    def _on_velocity(self, msg: TwistStamped) -> None:
        self._velocity = msg
        self._velocity_at_s = self._now_s()

    def _commanding(self) -> bool:
        """True when the pilot has handed over control and the drone is flying."""
        state = self._state
        return (
            state is not None
            and state.connected
            and state.armed
            and state.mode == GUIDED_MODE
        )

    def _position_and_velocity(
        self, now_s: float
    ) -> tuple[Coordinate, tuple[float, float, float]] | None:
        """Latest position and velocity, or None until position is fresh."""
        if self._fix is None or self._altitude_m is None:
            return None
        if (
            now_s - self._fix_at_s > _TELEMETRY_STALE_S
            or now_s - self._altitude_at_s > _TELEMETRY_STALE_S
        ):
            return None

        velocity = (0.0, 0.0, 0.0)
        if (
            self._velocity is not None
            and now_s - self._velocity_at_s <= _TELEMETRY_STALE_S
        ):
            linear = self._velocity.twist.linear
            velocity = (linear.x, linear.y, linear.z)

        position = Coordinate(
            lat=self._fix.latitude,
            lon=self._fix.longitude,
            alt=self._altitude_m,
        )
        return position, velocity


class CheckFenceBreach(FenceTelemetry, py_trees.behaviour.Behaviour):
    """
    Succeeds while the drone is breached or recovering, fails while clear.

    Also stages the recovery target on the blackboard for
    ``RecoverIntoGeofence``. Recovery only ends once the drone has stayed clear
    of the warning margins for ``SAFE_HOLD_S``, so it is not handed straight
    back to a mission that would fly it at the fence again.
    """

    def __init__(self, name: str = "CheckFenceBreach") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.GEOFENCE_TARGET,
            access=py_trees.common.Access.WRITE,
        )

        self._recovering = False
        self._clear_since_s: float | None = None

    def _stop_recovering(self) -> None:
        self._recovering = False
        self._clear_since_s = None

    def update(self) -> py_trees.common.Status:
        fence = self._fence()
        if fence is None or not self._commanding():
            self._stop_recovering()
            return py_trees.common.Status.FAILURE

        now_s = self._now_s()
        reading = self._position_and_velocity(now_s)
        if reading is None:
            # Keep recovering through a blind spot rather than resuming the
            # mission with no idea where the drone is.
            return (
                py_trees.common.Status.SUCCESS
                if self._recovering
                else py_trees.common.Status.FAILURE
            )

        position, velocity = reading
        if not self._recovering:
            if not fence.breached(position):
                return py_trees.common.Status.FAILURE
            self._recovering = True
            self._node.get_logger().warning(
                f"{self.name}: fence breached, taking over the mission"
            )
            send_to_ground(self._node, "ENG: fence breach - recovering")

        if not fence.near(position, velocity):
            if self._clear_since_s is None:
                self._clear_since_s = now_s
            elif now_s - self._clear_since_s >= SAFE_HOLD_S:
                self._stop_recovering()
                self._node.get_logger().info(
                    f"{self.name}: back inside the fence, resuming the mission"
                )
                send_to_ground(self._node, "ENG: back inside fence - resuming")
                return py_trees.common.Status.FAILURE
        else:
            self._clear_since_s = None

        self.blackboard.set(
            blackboard_keys.GEOFENCE_TARGET,
            fence.recovery_coordinate(position),
        )
        return py_trees.common.Status.SUCCESS


class WarnNearFence(FenceTelemetry, py_trees.behaviour.Behaviour):
    """
    Warns the pilot and ground while the drone is near the fence.

    Always returns FAILURE so the safety selector carries on to the mission:
    warnings never stop the mission, only breaches do. The STATUSTEXT is sent
    once per entry into the warning band rather than every tick.
    """

    def __init__(self, name: str = "WarnNearFence") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        super().setup(**kwargs)
        self._warned_near = False

    def update(self) -> py_trees.common.Status:
        fence = self._fence()
        reading = self._position_and_velocity(self._now_s())
        if fence is None or not self._commanding() or reading is None:
            self._warned_near = False
            return py_trees.common.Status.FAILURE

        position, velocity = reading
        if fence.near(position, velocity):
            clearance_m = fence.clearance_m(position)
            self._node.get_logger().warning(
                f"{self.name}: {clearance_m:.0f}m from the fence",
                throttle_duration_sec=5.0,
            )
            if not self._warned_near:
                send_to_ground(self._node, f"ENG: {clearance_m:.0f}m from fence")
                self._warned_near = True
        else:
            self._warned_near = False
        return py_trees.common.Status.FAILURE


class RecoverIntoGeofence(py_trees.behaviour.Behaviour):
    """
    Flies the drone to the recovery target staged by ``CheckFenceBreach``.

    Returns RUNNING while the target is being flown; the safety selector
    preempts this branch once ``CheckFenceBreach`` reports the drone clear.
    """

    def __init__(self, name: str = "RecoverIntoGeofence") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.GEOFENCE_TARGET,
            access=py_trees.common.Access.READ,
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]
        self._publisher = self._node.create_publisher(
            msg_type=CoordinateMsg,
            topic=_TARGET_TOPIC,
            qos_profile=10,
        )

    def update(self) -> py_trees.common.Status:
        try:
            target = self.blackboard.get(blackboard_keys.GEOFENCE_TARGET)
        except KeyError:
            self._node.get_logger().error(
                f"{self.name}: no recovery target staged"
            )
            return py_trees.common.Status.FAILURE

        self._publisher.publish(
            CoordinateMsg(lat=target.lat, lon=target.lon, alt=target.alt)
        )
        self.feedback_message = f"recovering to {target}"
        self._node.get_logger().warning(
            f"{self.name}: {self.feedback_message}",
            throttle_duration_sec=5.0,
        )
        return py_trees.common.Status.RUNNING
