#!/usr/bin/env python3
"""Receive-only AEAC traffic capture. Never sends telemetry or other frames."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from urllib.parse import quote

import websocket

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "comms"))
from comms.traffic import decode_message, parse_traffic  # noqa: E402


DEFAULT_URL = "wss://o61e21rvtd.execute-api.ca-central-1.amazonaws.com/prod"


def capture(output: Path, duration_s: float, url: str, token: str) -> int:
    """Capture only validated traffic fields; return 2 if no moving aircraft."""

    if not token:
        raise ValueError("AEAC_CONNECTION_TOKEN is required")
    if not url.startswith("wss://"):
        raise ValueError("only secure WebSocket URLs are accepted")
    if duration_s <= 0.0 or not math.isfinite(duration_s):
        raise ValueError("duration must be positive and finite")

    output.parent.mkdir(parents=True, exist_ok=True)
    authenticated_url = (
        f"{url}{'&' if '?' in url else '?'}Authorization={quote(token, safe='')}"
    )
    count = moving = 0
    previous: float | None = None
    intervals: list[float] = []
    deadline = time.monotonic() + duration_s
    try:
        connection = websocket.create_connection(authenticated_url, timeout=10.0)
        try:
            connection.settimeout(1.0)
            with output.open("w", encoding="utf-8") as destination:
                while time.monotonic() < deadline:
                    try:
                        raw = connection.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    message = decode_message(raw)
                    if message is None or message.get("event") != "traffic":
                        continue
                    received_s = time.monotonic()
                    aircraft, problems = parse_traffic(message)
                    if previous is not None:
                        intervals.append(received_s - previous)
                    previous = received_s
                    count += 1
                    moving += any(item.speed_mps > 0.0 for item in aircraft)
                    sample = {
                        "received_unix_s": time.time(),
                        "received_monotonic_s": received_s,
                        "aircraft": [
                            {
                                "aircraft_index": item.aircraft_index,
                                "latitude_deg": item.lat,
                                "longitude_deg": item.lon,
                                "altitude_agl_m": item.altitude_agl_m,
                                "speed_mps": item.speed_mps,
                                "heading_deg_true": item.direction_deg,
                                "horizontal_keepaway_m": item.horizontal_keep_away_m,
                                "vertical_keepaway_m": item.vertical_keep_away_m,
                            }
                            for item in aircraft
                        ],
                        "problems": problems,
                    }
                    destination.write(json.dumps(sample, separators=(",", ":")) + "\n")
        finally:
            connection.close()
    except (OSError, websocket.WebSocketException) as error:
        # WebSocket exceptions may contain the authenticated URL; never print it.
        print(
            f"AEAC receive-only connection failed: {type(error).__name__}",
            file=sys.stderr,
        )
        return 3

    print(
        json.dumps(
            {
                "snapshots": count,
                "moving_snapshots": moving,
                "min_interval_s": min(intervals) if intervals else None,
                "max_interval_s": max(intervals) if intervals else None,
                "capture_path": str(output),
            },
            sort_keys=True,
        )
    )
    return 0 if moving else 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-s", type=float, default=60.0)
    args = parser.parse_args()
    try:
        return capture(
            args.output,
            args.duration_s,
            os.environ.get("AEAC_WEBSOCKET_URL", "").strip() or DEFAULT_URL,
            os.environ.get("AEAC_CONNECTION_TOKEN", "").strip(),
        )
    except ValueError as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
