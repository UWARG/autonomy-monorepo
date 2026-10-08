"""Runs real multi-process fleets, against the fake groundside or with stub workers."""

from __future__ import annotations

import asyncio
import logging
import multiprocessing
import signal
import threading
import time
from collections.abc import Iterator
from multiprocessing.process import BaseProcess
from multiprocessing.synchronize import Event

import pytest
from websockets.asyncio.server import serve

from src.geometry import Vector3
from src.mock_fleet import SHUTDOWN_GRACE_S, FleetConfig, run_fleet
from tests.fake_groundside import FakeGroundside

# Stub workers. They run in child processes, so they must be module-level functions
# with drone_worker's signature.


def _crashing_worker(
    drone_id: str, url: str, start: Vector3, speed: float, period: float, level: str,
    stop_event: Event,
) -> None:  # fmt: skip
    raise RuntimeError(f"{drone_id} failed on purpose")


def _stubborn_worker(
    drone_id: str, url: str, start: Vector3, speed: float, period: float, level: str,
    stop_event: Event,
) -> None:  # fmt: skip
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    time.sleep(60)  # ignores stop_event, so the launcher has to terminate it


def _polite_worker(
    drone_id: str, url: str, start: Vector3, speed: float, period: float, level: str,
    stop_event: Event,
) -> None:  # fmt: skip
    stop_event.wait(30)


@pytest.fixture
def groundside_url() -> Iterator[tuple[FakeGroundside, str]]:
    """A FakeGroundside served from a background thread, so drone processes can
    connect while the test thread runs the fleet."""
    groundside = FakeGroundside(command_interval=0.1, area=5.0, seed=1)
    ready = threading.Event()
    port: list[int] = []
    loop = asyncio.new_event_loop()
    stopped = loop.create_future()

    async def serve_until_stopped() -> None:
        async with serve(groundside.handle, "127.0.0.1", 0) as server:
            port.append(server.sockets[0].getsockname()[1])
            ready.set()
            await stopped

    thread = threading.Thread(
        target=loop.run_until_complete, args=(serve_until_stopped(),), daemon=True
    )
    thread.start()
    assert ready.wait(5.0), "fake groundside did not start"
    try:
        yield groundside, f"ws://127.0.0.1:{port[0]}"
    finally:
        loop.call_soon_threadsafe(stopped.set_result, None)
        thread.join(5.0)
        if not thread.is_alive():  # closing a still-running loop would raise
            loop.close()


def test_fleet_drones_all_complete_tasks_and_stop_cleanly(
    groundside_url: tuple[FakeGroundside, str],
) -> None:
    groundside, url = groundside_url
    config = FleetConfig(num_drones=3, url=url, speed=50.0, report_period=0.05)
    stop_event = multiprocessing.Event()
    fleet_done = threading.Event()
    expected = {"drone-01", "drone-02", "drone-03"}

    def stop_once_every_drone_has_finished_a_task() -> None:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not fleet_done.is_set():
            if {drone for drone, _ in groundside.completed_tasks} >= expected:
                break
            time.sleep(0.05)
        stop_event.set()

    watcher = threading.Thread(target=stop_once_every_drone_has_finished_a_task)
    watcher.start()
    try:
        exit_code = run_fleet(config, stop_event)
    finally:
        fleet_done.set()  # don't leave the watcher polling if run_fleet raised
        watcher.join()

    finished = {drone for drone, _ in groundside.completed_tasks}
    assert finished >= expected, f"completed={groundside.completed_tasks}"
    assert groundside.rejected_tasks == []
    assert exit_code == 0


def test_fleet_reports_failure_when_drones_crash(
    caplog: pytest.LogCaptureFixture,
) -> None:
    config = FleetConfig(num_drones=2, url="ws://127.0.0.1:1", report_period=0.05)

    with caplog.at_level(logging.ERROR, logger="src.mock_fleet"):
        exit_code = run_fleet(config, multiprocessing.Event(), _crashing_worker)

    assert exit_code == 1
    assert "drone-01 exited with code 1" in caplog.text
    assert "drone-02 exited with code 1" in caplog.text
    assert "every drone has exited" in caplog.text


def test_drones_that_ignore_stop_are_terminated_within_one_shared_timeout(
    caplog: pytest.LogCaptureFixture,
) -> None:
    config = FleetConfig(num_drones=3, url="ws://127.0.0.1:1", report_period=0.05)
    timeout = config.report_period + SHUTDOWN_GRACE_S
    stop_event = multiprocessing.Event()
    threading.Timer(1.0, stop_event.set).start()  # let the processes start first

    with caplog.at_level(logging.ERROR, logger="src.mock_fleet"):
        started = time.monotonic()
        exit_code = run_fleet(config, stop_event, _stubborn_worker)
        elapsed = time.monotonic() - started

    assert exit_code == 1
    for drone_id in ("drone-01", "drone-02", "drone-03"):
        assert f"{drone_id} did not stop within" in caplog.text
    assert "exited with code" not in caplog.text  # terminations aren't double-logged
    # One shared deadline: well under 3 drones x timeout.
    assert elapsed < 1.0 + timeout + 2.0, f"shutdown took {elapsed:.1f}s"


def test_ctrl_c_during_startup_stops_the_drones_already_started(
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    original_start = BaseProcess.start
    starts = 0

    def start_then_interrupt(self: BaseProcess) -> None:
        nonlocal starts
        starts += 1
        if starts == 2:
            raise KeyboardInterrupt  # Ctrl+C while starting the second drone
        original_start(self)

    monkeypatch.setattr(BaseProcess, "start", start_then_interrupt)
    config = FleetConfig(num_drones=3, url="ws://127.0.0.1:1", report_period=0.05)

    with caplog.at_level(logging.INFO, logger="src.mock_fleet"):
        exit_code = run_fleet(config, multiprocessing.Event(), _polite_worker)

    assert exit_code == 0  # the one started drone stopped cleanly
    assert starts == 2  # the third drone was never started
    assert "Ctrl+C received" in caplog.text
    assert "fleet stopped cleanly" in caplog.text
