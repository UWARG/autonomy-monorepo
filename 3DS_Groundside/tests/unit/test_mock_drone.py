"""Unit tests for src.mock_drone that need no network connection."""

import asyncio
import logging
import math
from typing import Any

import pytest
from websockets.exceptions import InvalidURI

from src.geometry import Vector3
from src.messages import LocationCommand, MissionState
from src.mock_drone import MockDrone, _next_deadline
from tests.fakes import FakeClock


def _drone(clock: FakeClock) -> MockDrone:
    return MockDrone(
        "drone-03", start_position=(0.0, 0.0, 15.0), speed=2.0, clock=clock
    )


def _command(
    task_id: str = "task-17", target: Vector3 = (30.0, 40.0, 15.0)
) -> LocationCommand:
    return LocationCommand(task_id=task_id, target=target, yaw=1.57)


def test_new_drone_is_idle_with_no_task(fake_clock: FakeClock) -> None:
    payload = _drone(fake_clock).state_message()["payload"]

    assert payload["mission_state"] == "IDLE"
    assert payload["task_id"] is None
    assert payload["position"] == {"x": 0.0, "y": 0.0, "z": 15.0}


def test_idle_drone_accepts_command_and_starts_moving(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)

    assert drone.handle_command(_command()) is True
    assert drone.mission_state is MissionState.MOVING
    assert drone.task_id == "task-17"


def test_moving_drone_rejects_new_command_and_keeps_current_task(
    fake_clock: FakeClock,
) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command("task-17"))

    assert drone.handle_command(_command("task-18", target=(0.0, 0.0, 15.0))) is False
    assert drone.task_id == "task-17"
    fake_clock.advance(1.0)
    drone.update()  # still flies toward task-17's target, not task-18's
    assert drone.position == pytest.approx((1.2, 1.6, 15.0))


def test_resent_current_command_is_reacknowledged_without_restarting(
    fake_clock: FakeClock,
) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command("task-17"))
    fake_clock.advance(1.0)

    assert drone.handle_command(_command("task-17")) is True

    fake_clock.advance(1.0)
    drone.update()
    # 2 s of flight in total, so the leg was not restarted by the resend.
    assert drone.position == pytest.approx((2.4, 3.2, 15.0))


def test_resent_finished_command_is_reacknowledged_without_flying_again(
    fake_clock: FakeClock,
) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command("task-17", target=(2.0, 0.0, 15.0)))
    fake_clock.advance(1.0)
    drone.update()  # ARRIVED
    fake_clock.advance(1.0)
    drone.update()  # IDLE

    assert drone.handle_command(_command("task-17", target=(2.0, 0.0, 15.0))) is True

    fake_clock.advance(1.0)
    drone.update()
    # Stays IDLE rather than reporting task-17 as ARRIVED a second time.
    assert drone.mission_state is MissionState.IDLE


def test_reused_task_id_with_different_target_is_rejected_while_moving(
    fake_clock: FakeClock, caplog: pytest.LogCaptureFixture
) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command("task-17"))

    with caplog.at_level(logging.WARNING, logger="src.mock_drone"):
        reused = _command("task-17", target=(0.0, 0.0, 15.0))
        assert drone.handle_command(reused) is False

    assert "drone-03: rejecting task-17: task ID reused" in caplog.text
    fake_clock.advance(1.0)
    drone.update()  # still flying to the original target
    assert drone.position == pytest.approx((1.2, 1.6, 15.0))


def test_reused_task_id_with_different_target_is_rejected_after_finishing(
    fake_clock: FakeClock,
) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command("task-17", target=(2.0, 0.0, 15.0)))
    fake_clock.advance(1.0)
    drone.update()  # ARRIVED
    drone.update()  # IDLE

    assert drone.handle_command(_command("task-17", target=(9.0, 0.0, 15.0))) is False
    assert drone.mission_state is MissionState.IDLE


def test_arrived_drone_rejects_new_command(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command(target=(1.0, 0.0, 15.0)))
    fake_clock.advance(1.0)
    drone.update()
    assert drone.mission_state is MissionState.ARRIVED

    assert drone.handle_command(_command("task-18")) is False


def test_position_follows_elapsed_time_at_speed(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command())

    fake_clock.advance(1.0)
    drone.update()

    # 2 m along the 3-4-5 direction toward (30, 40).
    assert drone.position == pytest.approx((1.2, 1.6, 15.0))
    assert drone.mission_state is MissionState.MOVING


def test_position_does_not_depend_on_how_often_update_is_called(
    fake_clock: FakeClock,
) -> None:
    rarely, often = _drone(fake_clock), _drone(fake_clock)
    rarely.handle_command(_command())
    often.handle_command(_command())

    for _ in range(25):
        fake_clock.advance(0.1)
        often.update()
    rarely.update()

    # 5 m after 2.5 s either way, with no error building up between updates.
    assert rarely.position == pytest.approx((3.0, 4.0, 15.0))
    assert often.position == pytest.approx(rarely.position)


def test_drone_does_not_move_until_update_is_called(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command())

    fake_clock.advance(5.0)

    assert drone.position == (0.0, 0.0, 15.0)


def test_idle_drone_does_not_move(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)

    fake_clock.advance(10.0)
    drone.update()

    assert drone.position == (0.0, 0.0, 15.0)
    assert drone.mission_state is MissionState.IDLE


def test_full_mission_cycle_returns_to_idle_and_accepts_next_command(
    fake_clock: FakeClock,
) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command(target=(3.0, 0.0, 15.0)))

    fake_clock.advance(1.0)  # 2 m of 3 m
    drone.update()
    assert drone.mission_state is MissionState.MOVING
    fake_clock.advance(1.0)  # reaches target
    drone.update()
    assert drone.mission_state is MissionState.ARRIVED
    assert drone.position == (3.0, 0.0, 15.0)
    fake_clock.advance(
        1.0
    )  # ARRIVED is reported for one update, then the drone is free
    drone.update()
    assert drone.mission_state is MissionState.IDLE
    assert drone.task_id == "task-17"  # last task stays visible to groundside

    assert drone.handle_command(_command("task-18")) is True


def test_next_leg_starts_from_where_previous_leg_ended(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command(target=(4.0, 0.0, 15.0)))
    fake_clock.advance(2.0)
    drone.update()  # ARRIVED at (4, 0, 15)
    fake_clock.advance(1.0)
    drone.update()  # IDLE

    # Idle for a while; the new leg's timing starts when the command arrives.
    fake_clock.advance(30.0)
    drone.handle_command(_command("task-18", target=(4.0, 10.0, 15.0)))
    fake_clock.advance(1.0)
    drone.update()

    assert drone.position == pytest.approx((4.0, 2.0, 15.0))


def test_state_message_reports_yaw_from_command(fake_clock: FakeClock) -> None:
    drone = _drone(fake_clock)
    drone.handle_command(_command())

    orientation = drone.state_message()["payload"]["orientation"]

    assert orientation == {"roll": 0.0, "pitch": 0.0, "yaw": 1.57}


@pytest.mark.parametrize(
    "kwargs",
    [
        {"speed": 0.0},
        {"speed": -1.0},
        {"speed": math.nan},
        {"speed": math.inf},
        {"report_period": 0.0},
        {"report_period": math.nan},
        {"reconnect_min_delay": 0.0},
        {"reconnect_max_delay": math.inf},
        {"start_position": (math.nan, 0.0, 0.0)},
        {"reconnect_min_delay": 5.0, "reconnect_max_delay": 1.0},
    ],
)
def test_invalid_constructor_arguments_raise(kwargs: dict[str, Any]) -> None:
    arguments: dict[str, Any] = {"start_position": (0.0, 0.0, 0.0), "speed": 2.0}
    arguments.update(kwargs)

    with pytest.raises(ValueError):
        MockDrone("drone-03", **arguments)


def test_malformed_url_raises_instead_of_retrying() -> None:
    drone = MockDrone("drone-03", (0.0, 0.0, 0.0), speed=2.0)

    with pytest.raises(InvalidURI):
        asyncio.run(asyncio.wait_for(drone.run("ws//localhost:8765"), 2.0))


@pytest.mark.parametrize(
    ("previous", "now", "expected"),
    [
        (10.0, 10.2, 11.0),  # on schedule: one period after the last deadline
        (10.0, 10.9, 11.0),  # running late but within the period: keep the schedule
        (10.0, 15.0, 15.0),  # paused for several periods: restart instead of bursting
    ],
)
def test_next_deadline(previous: float, now: float, expected: float) -> None:
    assert _next_deadline(previous, 1.0, now) == expected
