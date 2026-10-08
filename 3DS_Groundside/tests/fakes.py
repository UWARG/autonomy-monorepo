"""Test doubles and builders shared by the unit and integration tests."""

from __future__ import annotations

from typing import Any, Callable

LocationCommandFactory = Callable[..., dict[str, Any]]


class FakeClock:
    """A clock that only moves when the test advances it."""

    def __init__(self, start: float = 100.0) -> None:
        # Not 0, so code that mistakes absolute time for elapsed time fails.
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def make_location_command(
    drone_id: str = "drone-03", **payload_overrides: Any
) -> dict[str, Any]:
    """A valid LOCATION_COMMAND dict; keyword arguments override payload fields."""
    payload: dict[str, Any] = {
        "task_id": "task-17",
        "target": {"x": 30.0, "y": 20.0, "z": 15.0},
        "orientation": {"yaw": 1.57},
    }
    payload.update(payload_overrides)
    return {
        "type": "LOCATION_COMMAND",
        "drone_id": drone_id,
        "timestamp": 1790293201.800,
        "payload": payload,
    }
