import types
from pathlib import Path

from airside_interfaces.msg import Coordinate as CoordinateMsg
from builtin_interfaces.msg import Time
from engine.fence import Fence
from mavros_msgs.msg import GlobalPositionTarget
from navigation.position_controller_node import PositionController

from utils.src.waypoint_utils import parse_geofence_file

FENCE_PATH = Path(__file__).parents[2] / "engine" / "config" / "geofence.yaml"
OUTSIDE_NOTCH = (43.4350, -80.5730)
INSIDE_NOTCH = (43.4350, -80.5765)


class _Logger:
    def error(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def debug(self, *args, **kwargs):
        pass


class _Clock:
    def now(self):
        return types.SimpleNamespace(to_msg=lambda: Time(sec=0, nanosec=0))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _controller(fence):
    controller = object.__new__(PositionController)
    controller._fence = fence
    controller._setpoint_pub = _Publisher()
    controller.get_logger = lambda: _Logger()
    controller.get_clock = lambda: _Clock()
    return controller


def _fence():
    return Fence(parse_geofence_file(FENCE_PATH))


def test_outside_target_is_rejected():
    controller = _controller(_fence())
    controller._target_callback(
        CoordinateMsg(lat=OUTSIDE_NOTCH[0], lon=OUTSIDE_NOTCH[1], alt=10.0)
    )
    assert controller._setpoint_pub.messages == []


def test_target_is_rejected_without_a_fence():
    controller = _controller(None)
    controller._target_callback(
        CoordinateMsg(lat=INSIDE_NOTCH[0], lon=INSIDE_NOTCH[1], alt=10.0)
    )
    assert controller._setpoint_pub.messages == []


def test_inside_target_is_forwarded():
    controller = _controller(_fence())
    controller._target_callback(
        CoordinateMsg(lat=INSIDE_NOTCH[0], lon=INSIDE_NOTCH[1], alt=10.0)
    )

    assert len(controller._setpoint_pub.messages) == 1
    message = controller._setpoint_pub.messages[0]
    assert (message.latitude, message.longitude, message.altitude) == (
        INSIDE_NOTCH[0],
        INSIDE_NOTCH[1],
        10.0,
    )
    assert message.coordinate_frame == GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
