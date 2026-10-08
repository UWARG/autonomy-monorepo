"""Launches a fleet of mock drones, one process each, connected to one groundside URL.

python -m src.mock_fleet --num-drones 5 --url ws://127.0.0.1:8765
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import math
import multiprocessing
import signal
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass
from multiprocessing.process import BaseProcess
from multiprocessing.synchronize import Event
from typing import Callable

from websockets.exceptions import InvalidURI
from websockets.uri import parse_uri

from src.geometry import Vector3
from src.mock_drone import MockDrone

logger = logging.getLogger(__name__)

POLL_INTERVAL_S = 0.5
# Drones poll their stop flag once per report, so allow a report period plus margin.
SHUTDOWN_GRACE_S = 2.0
LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR")


@dataclass(frozen=True)
class FleetConfig:
    num_drones: int
    url: str
    speed: float = 5.0
    spacing: float = 5.0
    altitude: float = 15.0
    report_period: float = 1.0
    log_level: str = "INFO"

    def validate(self) -> None:
        """Raise ValueError for settings that would make every drone fail."""
        if self.num_drones < 1:
            raise ValueError(f"num_drones must be at least 1, got {self.num_drones}")
        for name in ("speed", "report_period"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"{name} must be a positive finite number, got {value}"
                )
        if not math.isfinite(self.spacing) or self.spacing < 0:
            raise ValueError(
                f"spacing must be a non-negative number, got {self.spacing}"
            )
        if not math.isfinite(self.altitude):
            raise ValueError(f"altitude must be finite, got {self.altitude}")
        if self.log_level not in LOG_LEVELS:
            raise ValueError(
                f"log_level must be one of {LOG_LEVELS}, got {self.log_level}"
            )
        try:
            parse_uri(self.url)
        except InvalidURI as error:
            raise ValueError(f"invalid url: {error}") from None


def drone_ids(num_drones: int) -> list[str]:
    """Unique, zero-padded IDs that sort correctly: drone-01 ... drone-12."""
    width = max(2, len(str(num_drones)))
    return [f"drone-{n:0{width}d}" for n in range(1, num_drones + 1)]


def start_positions(num_drones: int, spacing: float, altitude: float) -> list[Vector3]:
    """Drones in a line along x, `spacing` m apart, so they aren't all equally close
    to every target."""
    return [(i * spacing, 0.0, altitude) for i in range(num_drones)]


def drone_worker(
    drone_id: str,
    url: str,
    start_position: Vector3,
    speed: float,
    report_period: float,
    log_level: str,
    stop_event: Event,
) -> None:
    """Process entry point: run one MockDrone until the fleet stops or the parent dies."""
    # Ctrl+C reaches every process in the console; let the parent stop us cleanly.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    _configure_logging(log_level)

    parent = multiprocessing.parent_process()

    def should_stop() -> bool:
        # Also stop if the launcher was killed, rather than retrying forever as an orphan.
        return stop_event.is_set() or (parent is not None and not parent.is_alive())

    drone = MockDrone(drone_id, start_position, speed, report_period=report_period)
    asyncio.run(drone.run(url, should_stop))


Worker = Callable[[str, str, Vector3, float, float, str, Event], None]


def run_fleet(
    config: FleetConfig, stop_event: Event, worker: Worker = drone_worker
) -> int:
    """Run the fleet until `stop_event` is set, Ctrl+C, or every drone has exited.

    Returns 0 if every drone exited cleanly, otherwise 1. `worker` is replaceable for
    tests and must be a module-level function so child processes can import it.
    """
    config.validate()
    ids = drone_ids(config.num_drones)
    positions = start_positions(config.num_drones, config.spacing, config.altitude)
    processes = [
        multiprocessing.Process(
            target=worker,
            name=drone_id,
            args=(
                drone_id,
                config.url,
                position,
                config.speed,
                config.report_period,
                config.log_level,
                stop_event,
            ),
        )
        for drone_id, position in zip(ids, positions)
    ]
    started: list[BaseProcess] = []
    failed: set[str] = set()
    try:
        # Inside the try so Ctrl+C during a slow startup still stops started drones.
        for process in processes:
            process.start()
            started.append(process)
        logger.info("started %d drones against %s", len(started), config.url)

        while not stop_event.is_set():
            _report_exits(started, failed)
            if not any(p.is_alive() for p in started):
                logger.error("every drone has exited")
                break
            stop_event.wait(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        logger.info("Ctrl+C received, stopping the fleet")
    finally:
        stop_event.set()
        timeout = config.report_period + SHUTDOWN_GRACE_S
        failed.update(_shutdown(started, timeout))

    _report_exits(started, failed)
    if failed:
        logger.error("%d drone(s) failed: %s", len(failed), ", ".join(sorted(failed)))
        return 1
    logger.info("fleet stopped cleanly")
    return 0


def _report_exits(processes: Sequence[BaseProcess], failed: set[str]) -> None:
    """Log each drone that exited with an error, once."""
    for process in processes:
        code = process.exitcode
        if code not in (None, 0) and process.name not in failed:
            failed.add(process.name)
            logger.error("%s exited with code %s", process.name, code)


def _shutdown(processes: Sequence[BaseProcess], timeout: float) -> set[str]:
    """Give drones `timeout` s in total to stop, then terminate the rest.

    Returns the names of the drones that had to be terminated.
    """
    deadline = time.monotonic() + timeout
    for process in processes:
        process.join(max(0.0, deadline - time.monotonic()))

    terminated: set[str] = set()
    for process in processes:
        if process.is_alive():
            logger.error(
                "%s did not stop within %.1fs, terminating", process.name, timeout
            )
            process.terminate()
            process.join()
            terminated.add(process.name)
    return terminated


def _configure_logging(level: str) -> None:
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    logging.getLogger("websockets").setLevel(logging.WARNING)


def parse_args(argv: list[str] | None = None) -> FleetConfig:
    parser = argparse.ArgumentParser(description="Run a fleet of mock drones.")
    parser.add_argument("--num-drones", type=int, default=3)
    parser.add_argument("--url", default="ws://127.0.0.1:8765")
    parser.add_argument("--speed", type=float, default=5.0, help="m/s")
    parser.add_argument("--spacing", type=float, default=5.0, help="m between drones")
    parser.add_argument("--altitude", type=float, default=15.0, help="m")
    parser.add_argument("--report-period", type=float, default=1.0, help="s")
    parser.add_argument("--log-level", default="INFO", choices=LOG_LEVELS)
    args = parser.parse_args(argv)

    config = FleetConfig(
        num_drones=args.num_drones,
        url=args.url,
        speed=args.speed,
        spacing=args.spacing,
        altitude=args.altitude,
        report_period=args.report_period,
        log_level=args.log_level,
    )
    try:
        config.validate()
    except ValueError as error:
        parser.error(str(error))
    return config


def main() -> int:
    config = parse_args()
    _configure_logging(config.log_level)
    return run_fleet(config, multiprocessing.Event())



if __name__ == "__main__":
    sys.exit(main())
