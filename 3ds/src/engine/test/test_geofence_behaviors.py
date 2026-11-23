import types
from pathlib import Path

import py_trees
import pytest
from airside_interfaces.msg import Coordinate as CoordinateMsg
from builtin_interfaces.msg import Time
from engine import blackboard_keys
from engine.behaviors.commands.wait_for_setpoint import WaitForSetpoint
from engine.behaviors.guided_mode_gate import PauseUnlessGuided
from engine.behaviors.safety_supervisor import (
    CheckFenceBreach,
    RecoverIntoGeofence,
    WarnNearFence,
)
from engine.constants import GUIDED_MODE
from engine.fence import RECOVERY_MARGIN_M, Fence
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from utils.src.waypoint_utils import parse_geofence_file

FENCE_PATH = Path(__file__).parents[1] / "config" / "geofence.yaml"
HOME = (43.4339, -80.5776)
NEAR_EDGE = (43.4339, -80.57195)
OUTSIDE_NOTCH = (43.4350, -80.5730)
INSIDE_NOTCH = (43.4350, -80.5765)
STATUS_TOPIC = "mavros/statustext/send"


def _blackboard():
    return py_trees.blackboard.Blackboard()


@pytest.fixture(autouse=True)
def reset_blackboard():
    _blackboard().clear()
    yield
    _blackboard().clear()


@pytest.fixture
def fence(reset_blackboard):
    fence = Fence(parse_geofence_file(FENCE_PATH))
    writer = py_trees.blackboard.Client(name="test_fence_writer")
    writer.register_key(
        key=blackboard_keys.GEOFENCE, access=py_trees.common.Access.WRITE
    )
    writer.set(blackboard_keys.GEOFENCE, fence)
    yield fence


class _Logger:
    def warning(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _Clock:
    def __init__(self):
        self.time_s = 100.0

    def now(self):
        return types.SimpleNamespace(
            nanoseconds=int(self.time_s * 1e9),
            to_msg=lambda: Time(sec=int(self.time_s), nanosec=0),
        )


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self._clock = _Clock()
        self._logger = _Logger()
        self._publishers = {}

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def create_subscription(self, *args, **kwargs):
        return object()

    def create_publisher(self, msg_type=None, topic=None, qos_profile=None):
        return self._publishers.setdefault(topic, _Publisher())


def _texts(node, topic=STATUS_TOPIC):
    publisher = node._publishers.get(topic)
    return [] if publisher is None else [message.text for message in publisher.messages]


def _feed(behavior, node, lat, lon, alt=10.0, velocity=(0.0, 0.0, 0.0)):
    state = State()
    state.mode = GUIDED_MODE
    state.connected = True
    state.armed = True
    behavior._on_state(state)

    fix = NavSatFix()
    fix.latitude = lat
    fix.longitude = lon
    behavior._on_fix(fix)
    behavior._on_altitude(Float64(data=alt))

    twist = TwistStamped()
    twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = velocity
    behavior._on_velocity(twist)


def _staged_target():
    reader = py_trees.blackboard.Client(name="test_target_reader")
    reader.register_key(
        key=blackboard_keys.GEOFENCE_TARGET, access=py_trees.common.Access.READ
    )
    return reader.get(blackboard_keys.GEOFENCE_TARGET)


def test_warn_near_fence_sends_once_per_band_entry(fence):
    node = _Node()
    warn = WarnNearFence()
    warn.setup(node=node)

    _feed(warn, node, *NEAR_EDGE)
    assert warn.update() == py_trees.common.Status.FAILURE
    texts = _texts(node)
    assert len(texts) == 1
    assert "from fence" in texts[0]

    warn.update()
    assert len(_texts(node)) == 1

    _feed(warn, node, *HOME)
    warn.update()
    assert len(_texts(node)) == 1

    _feed(warn, node, *NEAR_EDGE)
    warn.update()
    assert len(_texts(node)) == 2


def test_warning_never_succeeds_when_breached(fence):
    node = _Node()
    warn = WarnNearFence()
    warn.setup(node=node)

    _feed(warn, node, *OUTSIDE_NOTCH)
    assert warn.update() == py_trees.common.Status.FAILURE
    assert len(_texts(node)) == 1


def test_breach_takes_over_recovers_and_releases(fence):
    node = _Node()
    check = CheckFenceBreach()
    check.setup(node=node)

    _feed(check, node, *HOME)
    assert check.update() == py_trees.common.Status.FAILURE
    assert _texts(node) == []

    _feed(check, node, *NEAR_EDGE)
    assert check.update() == py_trees.common.Status.FAILURE
    assert _texts(node) == []

    node._clock.time_s = 101.0
    _feed(check, node, *OUTSIDE_NOTCH)
    assert check.update() == py_trees.common.Status.SUCCESS
    assert "ENG: fence breach - recovering" in _texts(node)

    target = _staged_target()
    assert fence.contains(target)
    assert fence.clearance_m(target) >= RECOVERY_MARGIN_M - 0.5

    recover = RecoverIntoGeofence()
    recover.setup(node=node)
    assert recover.update() == py_trees.common.Status.RUNNING
    published = node._publishers["position_controller/target"].messages[-1]
    assert (published.lat, published.lon, published.alt) == (
        target.lat,
        target.lon,
        target.alt,
    )

    # A blind spot must not hand control back to the mission.
    node._clock.time_s = 104.0
    assert check.update() == py_trees.common.Status.SUCCESS

    _feed(check, node, target.lat, target.lon, target.alt)
    assert check.update() == py_trees.common.Status.SUCCESS
    node._clock.time_s = 105.0
    _feed(check, node, target.lat, target.lon, target.alt)
    assert check.update() == py_trees.common.Status.SUCCESS

    node._clock.time_s = 108.0
    _feed(check, node, target.lat, target.lon, target.alt)
    assert check.update() == py_trees.common.Status.FAILURE
    assert "ENG: back inside fence - resuming" in _texts(node)


def test_setpoint_outside_fence_is_rejected(fence):
    node = _Node()
    wait = WaitForSetpoint()
    wait.setup(node=node)

    wait._setpoint_callback(
        CoordinateMsg(lat=OUTSIDE_NOTCH[0], lon=OUTSIDE_NOTCH[1], alt=10.0)
    )
    assert len(wait._pending) == 0
    assert "ENG: setpoint outside fence" in _texts(node)

    wait._setpoint_callback(
        CoordinateMsg(lat=INSIDE_NOTCH[0], lon=INSIDE_NOTCH[1], alt=10.0)
    )
    assert len(wait._pending) == 1

    wait._setpoint_callback(CoordinateMsg(lat=HOME[0], lon=HOME[1], alt=10.0))
    assert len(wait._pending) == 2


def test_safety_preemption_preserves_mission_progress():
    class _RunningMission(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__("mission")
            self.ticks = 0

        def update(self):
            self.ticks += 1
            return py_trees.common.Status.RUNNING

    class _SafetyBranch(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__("safety")
            self.active = False

        def update(self):
            return (
                py_trees.common.Status.RUNNING
                if self.active
                else py_trees.common.Status.FAILURE
            )

    def tick(tree):
        for _ in tree.tick():
            pass

    node = _Node()
    mission = _RunningMission()
    gate = PauseUnlessGuided(child=mission)
    gate.setup(node=node)
    state = State()
    state.mode = GUIDED_MODE
    state.connected = True
    state.armed = True
    gate._state_callback(state)

    safety = _SafetyBranch()
    tree = py_trees.composites.Selector(
        name="SafetySupervisor", memory=False, children=[safety, gate]
    )

    tick(tree)
    assert mission.ticks == 1

    safety.active = True
    tick(tree)
    assert gate.status == py_trees.common.Status.INVALID
    assert mission.status == py_trees.common.Status.RUNNING
    assert mission.ticks == 1

    safety.active = False
    tick(tree)
    assert mission.ticks == 2
    assert gate.status == py_trees.common.Status.RUNNING
