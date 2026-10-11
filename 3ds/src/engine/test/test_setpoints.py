import pytest
from airside_interfaces.msg import Coordinate as CoordinateMsg

from engine.behaviors.commands.wait_for_setpoint import setpoint_from_msg
from engine.constants import SETPOINT_MIN_ALTITUDE_M
from utils.src.types import Coordinate


def test_setpoint_from_msg():
    msg = CoordinateMsg(lat=43.43, lon=-80.57, alt=10.0)
    assert setpoint_from_msg(msg) == Coordinate(lat=43.43, lon=-80.57, alt=10.0)


@pytest.mark.parametrize(
    "lat, lon",
    [(90.1, 0.0), (-90.1, 0.0), (0.0, 180.1), (0.0, -180.1), (float("nan"), 0.0)],
)
def test_setpoint_from_msg_rejects_out_of_range(lat, lon):
    with pytest.raises(ValueError):
        setpoint_from_msg(CoordinateMsg(lat=lat, lon=lon, alt=10.0))


@pytest.mark.parametrize("alt", [SETPOINT_MIN_ALTITUDE_M - 0.1, 0.0, float("nan")])
def test_setpoint_from_msg_rejects_low_altitude(alt):
    with pytest.raises(ValueError):
        setpoint_from_msg(CoordinateMsg(lat=43.43, lon=-80.57, alt=alt))
