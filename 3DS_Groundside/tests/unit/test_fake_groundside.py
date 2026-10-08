"""Unit tests for the fake groundside's bookkeeping (no real network)."""

from __future__ import annotations

import asyncio
import logging
import math
from typing import Any, cast

import pytest
from websockets.asyncio.server import ServerConnection
from websockets.exceptions import ConnectionClosed

from tests.fake_groundside import FakeGroundside, _DroneView, _parse_register


class _FakeConnection:
    """Just enough of a ServerConnection for _register() and _command_loop()."""

    def __init__(
        self, send_error: Exception | None = None, close_hangs: bool = False
    ) -> None:
        self.closed = False
        self._send_error = send_error
        self._close_hangs = close_hangs

    async def close(self) -> None:
        if self._close_hangs:  # like a dead peer that never answers the close
            await asyncio.Event().wait()
        self.closed = True

    async def send(self, message: str) -> None:
        if self._send_error is not None:
            raise self._send_error


def _run_command_loop(connection: _FakeConnection) -> None:
    """Run the command loop for an available drone until it ends."""
    groundside = FakeGroundside(command_interval=0.01)
    websocket = cast(ServerConnection, connection)
    view = _DroneView(mission_state="IDLE")
    asyncio.run(
        asyncio.wait_for(groundside._command_loop("drone-01", view, websocket), 2.0)
    )


def _register_all(
    groundside: FakeGroundside, *connections: _FakeConnection
) -> list[_DroneView]:
    """Register each connection as drone-01 in turn, then let background closes run."""

    async def run() -> list[_DroneView]:
        views = [
            groundside._register("drone-01", cast(ServerConnection, c), {})
            for c in connections
        ]
        await asyncio.sleep(0.01)
        return views

    return asyncio.run(asyncio.wait_for(run(), 2.0))


def _ack(task_id: str, accepted: bool) -> dict[str, Any]:
    return {
        "type": "COMMAND_ACK",
        "drone_id": "drone-01",
        "timestamp": 0.0,
        "payload": {"task_id": task_id, "accepted": accepted},
    }


def test_new_idle_drone_is_available() -> None:
    assert _DroneView(mission_state="IDLE").available


def test_drone_is_not_available_until_idle_on_assigned_task() -> None:
    view = _DroneView(mission_state="IDLE", assigned_task="task-1")

    # Report sent before the drone processed task-1: still IDLE on its old task.
    view.task_id = None
    assert not view.available

    view.mission_state, view.task_id = "MOVING", "task-1"
    assert not view.available
    view.mission_state = "ARRIVED"
    assert not view.available
    view.mission_state = "IDLE"
    assert view.available


def test_drone_that_has_not_reported_yet_is_not_available() -> None:
    assert not _DroneView().available


def test_reconnect_starts_with_fresh_view_so_lost_command_cannot_starve_drone() -> None:
    first, second = _register_all(
        FakeGroundside(), _FakeConnection(), _FakeConnection()
    )
    first.assigned_task = "task-1"  # sent, but the connection dropped before delivery
    second.mission_state = "IDLE"  # drone never got task-1

    assert second.assigned_task is None
    assert second.available


def test_registering_again_closes_the_old_connection() -> None:
    old, new = _FakeConnection(), _FakeConnection()

    _register_all(FakeGroundside(), old, new)

    assert old.closed
    assert not new.closed


def test_registering_again_does_not_wait_for_a_dead_old_connection() -> None:
    # The old close never finishes; registration must still complete promptly
    # (_register_all fails with a timeout after 2 s otherwise).
    views = _register_all(
        FakeGroundside(), _FakeConnection(close_hangs=True), _FakeConnection()
    )

    assert len(views) == 2


def test_command_loop_crash_is_logged_not_silent(
    caplog: pytest.LogCaptureFixture,
) -> None:
    caplog.set_level(logging.ERROR, logger="fake_groundside")

    with pytest.raises(RuntimeError):
        _run_command_loop(_FakeConnection(send_error=RuntimeError("boom")))

    assert "drone-01 command loop crashed" in caplog.text
    assert "RuntimeError: boom" in caplog.text  # traceback included


def test_command_loop_ends_quietly_when_drone_disconnects(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level(logging.ERROR, logger="fake_groundside"):
        _run_command_loop(_FakeConnection(send_error=ConnectionClosed(None, None)))

    assert caplog.text == ""


def test_rejected_ack_frees_the_drone_and_is_recorded() -> None:
    groundside = FakeGroundside()
    view = _DroneView(mission_state="IDLE", task_id=None, assigned_task="task-1")

    groundside._on_message("drone-01", view, _ack("task-1", accepted=False))

    assert view.available
    assert groundside.rejected_tasks == [("drone-01", "task-1")]


@pytest.mark.parametrize(
    "raw",
    [
        "not json",
        "[]",
        '{"type": "DRONE_STATE"}',
        '{"type": "DRONE_STATE", "payload": 1}',
    ],
)
def test_malformed_message_is_logged_not_raised(
    raw: str, caplog: pytest.LogCaptureFixture
) -> None:
    groundside = FakeGroundside()

    with caplog.at_level(logging.WARNING, logger="fake_groundside"):
        groundside._on_raw_message("drone-01", _DroneView(), raw)

    assert "drone-01 sent a malformed message" in caplog.text


@pytest.mark.parametrize(
    "kwargs",
    [
        {"command_interval": 0.0},
        {"command_interval": -1.0},
        {"command_interval": math.nan},
        {"area": 0.0},
        {"altitude": math.nan},
    ],
)
def test_invalid_settings_raise(kwargs: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        FakeGroundside(**kwargs)


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ('{"type": "DRONE_STATE", "drone_id": "d", "payload": {}}', "did not start"),
        ("not json", "invalid first message"),
        ('{"type": "REGISTER"}', "invalid first message"),
        ('{"type": "REGISTER", "drone_id": 7, "payload": {}}', "invalid REGISTER"),
    ],
)
def test_bad_first_message_is_rejected_with_accurate_log(
    raw: str, expected: str, caplog: pytest.LogCaptureFixture
) -> None:
    with caplog.at_level(logging.WARNING, logger="fake_groundside"):
        assert _parse_register(raw) is None

    assert expected in caplog.text


def test_valid_register_is_parsed() -> None:
    raw = '{"type": "REGISTER", "drone_id": "drone-01", "payload": {"a": 1}}'

    assert _parse_register(raw) == ("drone-01", {"a": 1})
