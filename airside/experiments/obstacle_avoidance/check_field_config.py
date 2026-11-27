#!/usr/bin/env python3
"""Check local flight-test settings without printing the AEAC token."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from obstacle_avoidance.boundary import FlightBoundary
from server_traffic_replay import load_route, load_route_altitudes
from utils.src.types import Coordinate


def check_config(settings: dict[str, str], route_file: Path, mode: str) -> list[str]:
    """Return missing/invalid gates; geometry validity is not site approval."""

    problems: list[str] = []
    if not settings.get("AEAC_CONNECTION_TOKEN", "").strip():
        problems.append("AEAC_CONNECTION_TOKEN missing")
    if not settings.get("AEAC_UAV_ID", "").strip():
        problems.append("AEAC_UAV_ID missing")
    if not settings.get("AEAC_WEBSOCKET_URL", "").strip().startswith("wss://"):
        problems.append("AEAC_WEBSOCKET_URL must be wss://")
    boundary: FlightBoundary | None = None
    try:
        boundary = FlightBoundary.from_json(settings.get("FLIGHT_BOUNDARY_WGS84", ""))
    except ValueError:
        problems.append("FLIGHT_BOUNDARY_WGS84 missing or invalid")
    try:
        positions = load_route(route_file)
        altitudes = load_route_altitudes(route_file)
        route = tuple(
            Coordinate(lat, lon, alt)
            for (lat, lon), alt in zip(positions, altitudes, strict=True)
        )
        if len(route) < 3:
            raise ValueError
    except (OSError, KeyError, TypeError, ValueError):
        problems.append("route missing or invalid")
        route = ()
    if boundary is not None and route:
        if any(not boundary.allows_point_with_margin(p, 3.0) for p in route):
            problems.append("route waypoint outside inset flight boundary")
        if any(
            not boundary.allows_segment(a, b)
            for a, b in zip(route, route[1:] + route[:1])
        ):
            problems.append("route segment crosses inset flight boundary")
    if settings.get("USE_AEAC_TELEMETRY", "false").lower() != "true":
        problems.append("USE_AEAC_TELEMETRY must be true for authorized two-way test")
    engine_enabled = settings.get("USE_ENGINE", "true").lower() == "true"
    if mode == "props-off" and engine_enabled:
        problems.append("USE_ENGINE must be false for props-off test")
    if mode == "flight" and not engine_enabled:
        problems.append("USE_ENGINE must be true for flight")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", required=True, type=Path)
    parser.add_argument("--mode", required=True, choices=("props-off", "flight"))
    args = parser.parse_args()
    problems = check_config(dict(os.environ), args.route, args.mode)
    if problems:
        print("CONFIG_NOT_READY")
        for problem in problems:
            print(f"- {problem}")
        return 2
    print(
        "LOCAL_CONFIG_VALID_ONLY: confirm site approval, FCU fence, live latency and pilot takeover before flight"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
