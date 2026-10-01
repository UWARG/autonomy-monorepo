#!/usr/bin/env python3
"""Deterministic local AEAC traffic server for Airside qualification."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True, slots=True)
class FakeTrafficConfig:
    scenario: str
    own_aircraft_index: int = 1
    intruder_aircraft_index: int = 2
    horizontal_keepaway_m: float = 5.0
    vertical_keepaway_m: float = 5.0
    intruder_altitude_agl_m: float = 15.0

    def __post_init__(self) -> None:
        if self.scenario not in {
            "clear",
            "static",
            "crossing",
            "dropout",
            "malformed",
            "reconnect",
        }:
            raise ValueError("unsupported fake AEAC scenario")
        if self.own_aircraft_index < 0 or self.intruder_aircraft_index < 0:
            raise ValueError("aircraft indices must be non-negative")
        if self.own_aircraft_index == self.intruder_aircraft_index:
            raise ValueError("own and intruder indices must differ")
        if self.horizontal_keepaway_m <= 0.0 or self.vertical_keepaway_m <= 0.0:
            raise ValueError("keep-away values must be positive")
        if not math.isfinite(self.intruder_altitude_agl_m):
            raise ValueError("intruder altitude must be finite")


def offset_coordinate(
    latitude_deg: float,
    longitude_deg: float,
    east_m: float,
    north_m: float,
) -> tuple[float, float]:
    latitude = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = longitude_deg + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_deg)))
    )
    return latitude, longitude


def east_north_offset_m(
    from_latitude_deg: float,
    from_longitude_deg: float,
    to_latitude_deg: float,
    to_longitude_deg: float,
) -> tuple[float, float]:
    east_m = (
        math.radians(to_longitude_deg - from_longitude_deg)
        * math.cos(math.radians((from_latitude_deg + to_latitude_deg) / 2.0))
        * EARTH_RADIUS_M
    )
    north_m = math.radians(to_latitude_deg - from_latitude_deg) * EARTH_RADIUS_M
    return east_m, north_m


def traffic_payload(
    *,
    config: FakeTrafficConfig,
    own_telemetry: dict[str, Any],
    elapsed_s: float,
    origin_telemetry: dict[str, Any] | None = None,
    crossing_elapsed_s: float | None = None,
) -> dict[str, Any]:
    """Build one complete AEAC-style traffic snapshot."""

    latitude = float(own_telemetry["latitude"])
    longitude = float(own_telemetry["longitude"])
    altitude = float(own_telemetry["altitudeAGL"])
    origin = origin_telemetry or own_telemetry
    origin_latitude = float(origin["latitude"])
    origin_longitude = float(origin["longitude"])
    if config.scenario == "clear":
        return {"event": "traffic", "payload": {"traffic": []}}

    traffic: list[dict[str, Any]] = [
        {
            "aircraftIndex": config.own_aircraft_index,
            "name": str(own_telemetry.get("uavId", "OWN")),
            "position": {
                "lat": latitude,
                "lon": longitude,
                "altitude": altitude,
                "speed": 0.0,
                "direction": 0.0,
            },
            "horizontalKeepAway": 1.0,
            "verticalKeepAway": 1.0,
        }
    ]
    if config.scenario != "crossing":
        intruder_east_m = 0.0
        speed_mps = 0.0
        heading_deg = 0.0
    else:
        heading_deg = 90.0
        speed_mps = 4.0 if crossing_elapsed_s is not None else 0.0
        intruder_east_m = -12.0 + speed_mps * (crossing_elapsed_s or 0.0)
    intruder_latitude, intruder_longitude = offset_coordinate(
        origin_latitude,
        origin_longitude,
        intruder_east_m,
        20.0,
    )
    traffic.append(
        {
            "aircraftIndex": config.intruder_aircraft_index,
            "name": "SIM-INTRUDER",
            "position": {
                "lat": intruder_latitude,
                "lon": intruder_longitude,
                "altitude": config.intruder_altitude_agl_m,
                "speed": speed_mps,
                "direction": heading_deg,
            },
            "horizontalKeepAway": config.horizontal_keepaway_m,
            "verticalKeepAway": config.vertical_keepaway_m,
        }
    )
    return {"event": "traffic", "payload": {"traffic": traffic}}


class FakeAeacServer:
    def __init__(self, config: FakeTrafficConfig, transcript_path: Path | None) -> None:
        self.config = config
        self.transcript_path = transcript_path
        self.stop = asyncio.Event()
        self.fault_start_s: float | None = None
        self.reconnect_done = False

    def record(self, direction: str, payload: dict[str, Any]) -> None:
        if self.transcript_path is None:
            return
        self.transcript_path.parent.mkdir(parents=True, exist_ok=True)
        record = {
            "monotonic_s": time.monotonic(),
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "direction": direction,
            "payload": payload,
        }
        with self.transcript_path.open("a", encoding="utf-8") as transcript:
            transcript.write(json.dumps(record, sort_keys=True) + "\n")

    async def handler(self, websocket: Any) -> None:
        latest_telemetry: dict[str, Any] | None = None
        origin_telemetry: dict[str, Any] | None = None
        first_telemetry_s: float | None = None
        crossing_start_s: float | None = None

        async def receive() -> None:
            nonlocal latest_telemetry, origin_telemetry, first_telemetry_s
            nonlocal crossing_start_s
            async for raw in websocket:
                try:
                    envelope = json.loads(raw)
                    if envelope.get("action") != "telemetry":
                        continue
                    telemetry = envelope.get("data")
                    if not isinstance(telemetry, dict):
                        continue
                    latest_telemetry = telemetry
                    if first_telemetry_s is None:
                        first_telemetry_s = time.monotonic()
                        origin_telemetry = dict(telemetry)
                    if (
                        self.config.scenario == "crossing"
                        and crossing_start_s is None
                        and origin_telemetry is not None
                    ):
                        _east_m, north_m = east_north_offset_m(
                            float(origin_telemetry["latitude"]),
                            float(origin_telemetry["longitude"]),
                            float(telemetry["latitude"]),
                            float(telemetry["longitude"]),
                        )
                        if north_m >= 8.0:
                            crossing_start_s = time.monotonic()
                    if (
                        self.config.scenario
                        in {"dropout", "malformed", "reconnect"}
                        and self.fault_start_s is None
                        and origin_telemetry is not None
                    ):
                        _east_m, north_m = east_north_offset_m(
                            float(origin_telemetry["latitude"]),
                            float(origin_telemetry["longitude"]),
                            float(telemetry["latitude"]),
                            float(telemetry["longitude"]),
                        )
                        if north_m >= 5.0:
                            self.fault_start_s = time.monotonic()
                    self.record("rx", envelope)
                except (TypeError, json.JSONDecodeError):
                    continue

        receiver = asyncio.create_task(receive())
        try:
            while not self.stop.is_set() and not receiver.done():
                if latest_telemetry is not None and first_telemetry_s is not None:
                    fault_age_s = (
                        time.monotonic() - self.fault_start_s
                        if self.fault_start_s is not None
                        else None
                    )
                    if (
                        self.config.scenario == "dropout"
                        and fault_age_s is not None
                        and fault_age_s < 4.0
                    ):
                        await asyncio.sleep(1.0)
                        continue
                    if (
                        self.config.scenario == "reconnect"
                        and fault_age_s is not None
                        and not self.reconnect_done
                    ):
                        self.reconnect_done = True
                        await websocket.close(code=1012, reason="simulated reconnect")
                        return
                    if (
                        self.config.scenario == "malformed"
                        and fault_age_s is not None
                        and fault_age_s < 4.0
                    ):
                        malformed = {
                            "event": "traffic",
                            "payload": {"traffic": "invalid"},
                        }
                        await websocket.send(json.dumps(malformed))
                        self.record("tx", malformed)
                        await asyncio.sleep(1.0)
                        continue
                    payload = traffic_payload(
                        config=self.config,
                        own_telemetry=latest_telemetry,
                        elapsed_s=time.monotonic() - first_telemetry_s,
                        origin_telemetry=origin_telemetry,
                        crossing_elapsed_s=(
                            time.monotonic() - crossing_start_s
                            if crossing_start_s is not None
                            else None
                        ),
                    )
                    await websocket.send(json.dumps(payload))
                    self.record("tx", payload)
                await asyncio.sleep(1.0)
        finally:
            receiver.cancel()
            try:
                await receiver
            except asyncio.CancelledError:
                pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--scenario",
        choices=(
            "clear",
            "static",
            "crossing",
            "dropout",
            "malformed",
            "reconnect",
        ),
        required=True,
    )
    parser.add_argument("--own-aircraft-index", type=int, default=1)
    parser.add_argument("--transcript")
    return parser.parse_args()


async def run(args: argparse.Namespace) -> None:
    from websockets.asyncio.server import serve

    server = FakeAeacServer(
        FakeTrafficConfig(
            scenario=args.scenario,
            own_aircraft_index=args.own_aircraft_index,
        ),
        Path(args.transcript) if args.transcript else None,
    )
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, server.stop.set)
        except NotImplementedError:
            pass
    async with serve(server.handler, args.host, args.port):
        await server.stop.wait()


def main() -> None:
    asyncio.run(run(parse_args()))


if __name__ == "__main__":
    main()
