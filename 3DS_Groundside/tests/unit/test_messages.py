"""Unit tests for src.messages."""

import json
import logging
import math
import time
from typing import Any

import pytest

from src.messages import (
    LocationCommand,
    MessageType,
    MissionState,
    command_ack,
    drone_state,
    encode,
    make_message,
    parse_location_command,
    register,
)
from tests.fakes import make_location_command


def test_make_message_has_envelope_and_current_timestamp() -> None:
    before = time.time()
    message = make_message(MessageType.REGISTER, "drone-03", {"a": 1})
    after = time.time()

    assert message["type"] == "REGISTER"
    assert message["drone_id"] == "drone-03"
    assert message["payload"] == {"a": 1}
    assert before <= message["timestamp"] <= after


def test_register_matches_spec() -> None:
    message = register("drone-03")

    assert message["type"] == "REGISTER"
    assert message["drone_id"] == "drone-03"
    assert message["payload"] == {"vehicle_type": "QUADCOPTER"}


@pytest.mark.parametrize("accepted", [True, False])
def test_command_ack_matches_spec(accepted: bool) -> None:
    message = command_ack("drone-03", "task-17", accepted)

    assert message["type"] == "COMMAND_ACK"
    assert message["payload"] == {"task_id": "task-17", "accepted": accepted}


def test_drone_state_matches_spec() -> None:
    message = drone_state(
        "drone-03",
        "task-17",
        MissionState.MOVING,
        position=(30.0, 20.0, 15.0),
        orientation=(1.53, 2.12, 1.57),
    )

    assert message["type"] == "DRONE_STATE"
    assert message["payload"] == {
        "task_id": "task-17",
        "mission_state": "MOVING",
        "position": {"x": 30.0, "y": 20.0, "z": 15.0},
        "orientation": {"roll": 1.53, "pitch": 2.12, "yaw": 1.57},
    }


def test_drone_state_before_first_command_has_null_task_id() -> None:
    message = drone_state(
        "drone-03", None, MissionState.IDLE, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0)
    )

    assert json.loads(encode(message))["payload"]["task_id"] is None


def test_drone_state_sends_integer_inputs_as_floats() -> None:
    message = drone_state("drone-03", None, MissionState.IDLE, (1, 2, 3), (0, 0, 0))

    payload = json.loads(encode(message))["payload"]
    numbers = [*payload["position"].values(), *payload["orientation"].values()]
    assert all(isinstance(n, float) for n in numbers)


def test_encode_round_trips_built_messages() -> None:
    for message in (
        register("drone-03"),
        command_ack("drone-03", "task-17", True),
        drone_state(
            "drone-03",
            "task-17",
            MissionState.ARRIVED,
            (1.0, 2.0, 3.0),
            (0.0, 0.0, 0.0),
        ),
    ):
        assert json.loads(encode(message)) == message


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf])
def test_encode_rejects_non_finite_numbers(bad_value: float) -> None:
    message = drone_state(
        "drone-03",
        "task-17",
        MissionState.MOVING,
        (bad_value, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )

    with pytest.raises(ValueError):
        encode(message)


def _command(drone_id: str = "drone-03", **payload_overrides: Any) -> str:
    return json.dumps(make_location_command(drone_id, **payload_overrides))


def test_parse_valid_location_command() -> None:
    assert parse_location_command(_command(), "drone-03") == LocationCommand(
        task_id="task-17", target=(30.0, 20.0, 15.0), yaw=1.57
    )


def test_parse_accepts_bytes_and_integer_coordinates() -> None:
    raw = _command(target={"x": 30, "y": 20, "z": 15}).encode()

    command = parse_location_command(raw, "drone-03")

    assert command is not None
    assert command.target == (30.0, 20.0, 15.0)


def test_parse_logs_why_it_ignored_a_command(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.DEBUG, logger="src.messages"):
        assert parse_location_command(_command("drone-03"), "drone-01") is None

    assert "addressed to 'drone-03'" in caplog.text


@pytest.mark.parametrize(
    "raw",
    [
        "not json",
        "[1, 2, 3]",
        "null",
        b"\xff\xfe",
        _command("drone-01"),  # for another drone
        json.dumps({**make_location_command(), "type": "DRONE_STATE"}),
        _command(task_id=None),
        _command(task_id=""),
        _command(task_id=17),
        _command(target=None),
        _command(target={"x": 30.0, "y": 20.0}),
        _command(target={"x": "30", "y": 20.0, "z": 15.0}),
        _command(target={"x": True, "y": 20.0, "z": 15.0}),
        # Python's json accepts NaN literals, so they can reach the parser.
        _command(target={"x": math.nan, "y": 20.0, "z": 15.0}),
        _command(orientation={}),
        _command(orientation={"yaw": "north"}),
    ],
)
def test_parse_rejects_invalid_messages(raw: Any) -> None:
    assert parse_location_command(raw, "drone-03") is None
