"""Unit tests for the fake groundside's bookkeeping (no real network)."""

from __future__ import annotations

import asyncio
import logging
import math
from typing import cast

import pytest
from websockets.asyncio.server import ServerConnection
from websockets.exceptions import ConnectionClosed

from tests.fake_groundside import FakeGroundside, _DroneView


class _FailingConnection:
    """A connection whose send() raises `error`."""

    def __init__(self, error: Exception) -> None:
        self._error = error

    async def send(self, message: str) -> None:
        raise self._error


def _run_command_loop(error: Exception) -> None:
    """Run the command loop for an available drone whose send() raises `error`."""
    groundside = FakeGroundside(command_interval=0.01)
    websocket = cast(ServerConnection, _FailingConnection(error))
    view = _DroneView(mission_state="IDLE")
    asyncio.run(
        asyncio.wait_for(groundside._command_loop("drone-01", view, websocket), 2.0)
    )


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


def test_rejected_ack_frees_the_drone_and_is_recorded() -> None:
    groundside = FakeGroundside()
    view = _DroneView(mission_state="IDLE", task_id=None, assigned_task="task-1")
    ack = {
        "type": "COMMAND_ACK",
        "drone_id": "drone-01",
        "timestamp": 0.0,
        "payload": {"task_id": "task-1", "accepted": False},
    }

    groundside._on_message("drone-01", view, ack)

    assert view.available
    assert groundside.rejected_tasks == [("drone-01", "task-1")]


def test_command_loop_crash_is_logged_not_silent(
    caplog: pytest.LogCaptureFixture,
) -> None:
    caplog.set_level(logging.ERROR, logger="fake_groundside")

    with pytest.raises(RuntimeError):
        _run_command_loop(RuntimeError("boom"))

    assert "drone-01 command loop crashed" in caplog.text
    assert "RuntimeError: boom" in caplog.text  # traceback included


def test_command_loop_ends_quietly_when_drone_disconnects(
    caplog: pytest.LogCaptureFixture,
) -> None:
    with caplog.at_level(logging.ERROR, logger="fake_groundside"):
        _run_command_loop(ConnectionClosed(None, None))

    assert caplog.text == ""


@pytest.mark.parametrize("interval", [0.0, -1.0, math.nan])
def test_invalid_command_interval_raises(interval: float) -> None:
    with pytest.raises(ValueError):
        FakeGroundside(command_interval=interval)
