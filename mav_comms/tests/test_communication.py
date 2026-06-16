from types import SimpleNamespace
from unittest.mock import MagicMock

from mav_comms.communication import MavComms
from utils import AttitudeMessage


def _make_comms(recv_return):
    conn = MagicMock()
    conn.master.recv_match.return_value = recv_return
    return MavComms(connection=conn), conn


def test_receive_attitude_returns_populated_message():
    fake_msg = SimpleNamespace(
        time_boot_ms=12345,
        roll=0.5,
        pitch=-0.25,
        yaw=1.0,
        rollspeed=0.1,
        pitchspeed=0.2,
        yawspeed=0.3,
    )
    comms, conn = _make_comms(fake_msg)

    result = comms.receive_attitude()

    assert result == AttitudeMessage(
        time_boot_ms=12345,
        roll=0.5,
        pitch=-0.25,
        yaw=1.0,
        rollspeed=0.1,
        pitchspeed=0.2,
        yawspeed=0.3,
    )
    conn.master.recv_match.assert_called_once_with(
        type="ATTITUDE",
        blocking=True,
        timeout=1.0,
    )


def test_receive_attitude_returns_none_on_timeout():
    comms, _ = _make_comms(recv_return=None)

    assert comms.receive_attitude() is None
