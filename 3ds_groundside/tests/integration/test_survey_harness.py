"""
Checks the end-to-end survey harness itself: real MockDrones over websockets,
the test drone manager, kills, and the synthetic photos.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Iterator

import pytest
from utils.src.types import Attitude, Coordinate

from src.groundside_manager.groundside_manager import MissionState
from src.image_stitcher.message import ImageMessage
from src.mock_drone_fleet.geometry import from_local
from src.survey_executive import SurveyExecutive, SurveyRunner, SurveyState
from tests.integration.survey_harness import (
    DEFAULT_HOME,
    FleetHarness,
    flying,
    photo_from_state,
)
from tests.unit.survey_executive.factories import make_drone, make_request

DRONES = ("drone-01", "drone-02")


@pytest.fixture
def fleet() -> Iterator[FleetHarness]:
    with flying(DRONES) as harness:
        yield harness


def target_north(metres: float, alt: float = 25.0) -> Coordinate:
    point = from_local(DEFAULT_HOME, (0.0, metres, alt))
    return Coordinate(point.latitude, point.longitude, point.altitude)


def arrived_on(fleet: FleetHarness, drone_id: str, task_id: str) -> bool:
    state = fleet.manager.get_all_drones()[drone_id]
    return (
        state.mission_state is MissionState.ARRIVED and state.active_task_id == task_id
    )


def test_drones_register_and_keep_reporting(fleet: FleetHarness) -> None:
    first = fleet.manager.get_all_drones()

    assert set(first) == set(DRONES)
    for state in first.values():
        assert state.connected
        assert state.vehicle_type == "QUADCOPTER"
        assert state.mission_state is MissionState.IDLE
        assert state.position is not None
        assert state.position.lat == pytest.approx(DEFAULT_HOME.latitude, abs=1e-4)
    fleet.wait_for(
        lambda: all(
            state.last_update_timestamp != first[drone_id].last_update_timestamp
            for drone_id, state in fleet.manager.get_all_drones().items()
        ),
        what="a second report from every drone",
    )


def test_command_is_acked_and_flown_to_arrival(fleet: FleetHarness) -> None:
    target = target_north(20.0)

    assert fleet.manager.send_location_command_sync("drone-01", "t-1", target, 0.0)

    fleet.wait_for(lambda: arrived_on(fleet, "drone-01", "t-1"), what="arrival")
    state = fleet.manager.get_all_drones()["drone-01"]
    assert state.position == target
    assert ("drone-01", "t-1", True) in fleet.manager.acks
    # The other drone never got a command.
    assert fleet.manager.get_all_drones()["drone-02"].active_task_id is None


def test_each_arrival_makes_one_photo_in_degrees(fleet: FleetHarness) -> None:
    sunk: list[ImageMessage] = []
    fleet.photo_sink = sunk.append
    target = target_north(15.0)

    fleet.manager.send_location_command_sync("drone-01", "t-1", target, math.pi / 2)
    fleet.wait_for(lambda: len(fleet.photos) == 1, what="the first photo")
    # The drone keeps reporting ARRIVED on t-1; that must not add photos.
    time.sleep(0.2)
    assert len(fleet.photos) == 1
    photo = fleet.photos[0]
    assert (photo.drone_id, photo.image_id) == ("drone-01", "t-1")
    assert (photo.lat, photo.lon, photo.alt) == (target.lat, target.lon, target.alt)
    assert photo.yaw_deg == pytest.approx(90.0)
    assert (photo.roll_deg, photo.pitch_deg) == (0.0, 0.0)
    assert photo.has_valid_position() and photo.has_valid_image()

    fleet.manager.send_location_command_sync(
        "drone-01", "t-2", target_north(30.0), -math.pi / 4
    )
    fleet.wait_for(lambda: len(fleet.photos) == 2, what="the second photo")
    assert fleet.photos[1].image_id == "t-2"
    assert fleet.photos[1].yaw_deg == pytest.approx(-45.0)
    assert sunk == fleet.photos
    assert fleet.held == []


def test_photos_keep_photo_refuses_are_held_not_sunk(fleet: FleetHarness) -> None:
    sunk: list[ImageMessage] = []
    fleet.photo_sink = sunk.append
    fleet.keep_photo = lambda photo: photo.drone_id != "drone-02"

    fleet.manager.send_location_command_sync("drone-01", "a", target_north(10.0), 0.0)
    fleet.manager.send_location_command_sync("drone-02", "b", target_north(10.0), 0.0)
    fleet.wait_for(lambda: len(fleet.photos) == 2, what="both photos")

    assert [photo.image_id for photo in sunk] == ["a"]
    assert [photo.image_id for photo in fleet.held] == ["b"]


def test_killed_drone_disconnects_and_goes_quiet(fleet: FleetHarness) -> None:
    fleet.kill("drone-02")

    fleet.wait_for(
        lambda: not fleet.manager.get_all_drones()["drone-02"].connected,
        what="drone-02 to disconnect",
    )
    stamp = fleet.manager.get_all_drones()["drone-02"].last_update_timestamp
    live_stamp = fleet.manager.get_all_drones()["drone-01"].last_update_timestamp
    time.sleep(0.2)
    assert fleet.manager.get_all_drones()["drone-02"].last_update_timestamp == stamp
    fleet.wait_for(
        lambda: fleet.manager.get_all_drones()["drone-01"].last_update_timestamp
        != live_stamp,
        what="drone-01 to keep reporting",
    )
    assert fleet.manager.get_all_drones()["drone-01"].connected
    assert not fleet.manager.send_location_command_sync(
        "drone-02", "t-1", target_north(10.0), 0.0
    )


def test_send_to_an_unknown_drone_returns_false(fleet: FleetHarness) -> None:
    assert not fleet.manager.send_location_command_sync(
        "drone-99", "t-1", target_north(10.0), 0.0
    )


def test_send_from_the_event_loop_is_refused(fleet: FleetHarness) -> None:
    errors: list[BaseException] = []
    done = threading.Event()

    def send_from_loop(photo: ImageMessage) -> None:
        try:
            fleet.manager.send_location_command_sync(
                "drone-01", "again", target_north(5.0), 0.0
            )
        except RuntimeError as error:
            errors.append(error)
        done.set()

    fleet.photo_sink = send_from_loop
    fleet.manager.send_location_command_sync("drone-01", "t-1", target_north(5.0), 0.0)

    assert done.wait(10.0)
    assert len(errors) == 1


def test_photo_from_state_needs_a_report_and_a_task() -> None:
    drone = make_drone(position=Coordinate(43.0, -80.0, 20.0))
    with pytest.raises(ValueError, match="position"):
        photo_from_state(drone)
    drone.orientation = Attitude(0.1, -0.2, math.pi, 0.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="task"):
        photo_from_state(drone)

    drone.active_task_id = "t-9"
    photo = photo_from_state(drone)

    assert photo.roll_deg == pytest.approx(math.degrees(0.1))
    assert photo.pitch_deg == pytest.approx(math.degrees(-0.2))
    assert photo.yaw_deg == pytest.approx(180.0)
    assert photo.image_id == "t-9"


def test_executive_starts_and_flies_against_the_harness(fleet: FleetHarness) -> None:
    executive = SurveyExecutive(
        make_request(),
        fleet.manager,
        settle_s=0.3,
        ack_timeout_s=0.5,
        dropout_after_s=1.0,
    )
    fleet.photo_sink = executive.on_photo
    executive.start()
    runner = SurveyRunner(executive, rate_hz=50.0)
    runner.start()
    try:
        fleet.wait_for(
            lambda: (
                (progress := executive.progress()).photos_received >= len(DRONES)
                and progress.missions_done >= len(DRONES)
            ),
            what="the executive to finish and receive a photo per drone",
        )
        assert executive.state is SurveyState.SURVEYING
        # Every command the drones saw was ours and accepted.
        assert all(accepted for _, _, accepted in fleet.manager.acks)
    finally:
        executive.abort("test over")
        runner.stop()
    assert executive.state is SurveyState.ABORTED
