#!/usr/bin/env python3
"""Verify the external, sanitized AEAC protocol-gate evidence.

This deliberately does not ship a production fixture in the repository. A
capture from the AEAC test server and its review metadata must be supplied to
the formal campaign explicitly; synthetic fixtures cannot satisfy this gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from aeac_bridge.protocol import parse_traffic_event


class FixtureVerificationError(ValueError):
    """Raised when protocol evidence is missing or internally inconsistent."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise FixtureVerificationError(f"cannot read JSON object: {path}") from error
    if not isinstance(value, dict):
        raise FixtureVerificationError(f"expected JSON object: {path}")
    return value


def verify_fixture(event_path: Path, metadata_path: Path) -> dict[str, Any]:
    event_text = event_path.read_text(encoding="utf-8")
    event = parse_traffic_event(event_text)
    if event is None:
        raise FixtureVerificationError("fixture is not a traffic event")

    metadata = _load_object(metadata_path)
    required_exact = {
        "source": "AEAC test server",
        "sanitized": True,
        "snapshot_semantics": "complete",
        "single_bidirectional_connection_verified": True,
    }
    for key, expected in required_exact.items():
        if metadata.get(key) != expected:
            raise FixtureVerificationError(f"metadata.{key} must equal {expected!r}")

    for key in (
        "captured_at_utc",
        "server_timestamp_semantics",
        "heartbeat_semantics",
        "disconnect_semantics",
    ):
        if not isinstance(metadata.get(key), str) or not metadata[key].strip():
            raise FixtureVerificationError(f"metadata.{key} must be documented")

    identity = metadata.get("own_aircraft_identity")
    if not isinstance(identity, dict):
        raise FixtureVerificationError("metadata.own_aircraft_identity is required")
    uav_id = identity.get("uavId")
    aircraft_index = identity.get("aircraftIndex")
    if not isinstance(uav_id, str) or not uav_id.strip():
        raise FixtureVerificationError("own identity uavId must be non-empty")
    if (
        isinstance(aircraft_index, bool)
        or not isinstance(aircraft_index, int)
        or aircraft_index < 0
    ):
        raise FixtureVerificationError(
            "own identity aircraftIndex must be a non-negative integer"
        )

    fixture_indices = {aircraft.aircraft_index for aircraft in event.aircraft}
    self_included = metadata.get("server_includes_own_aircraft")
    if not isinstance(self_included, bool):
        raise FixtureVerificationError(
            "metadata.server_includes_own_aircraft must be boolean"
        )
    if self_included and aircraft_index not in fixture_indices:
        raise FixtureVerificationError(
            "fixture does not contain the documented own aircraftIndex"
        )
    if not self_included and aircraft_index in fixture_indices:
        raise FixtureVerificationError(
            "fixture contains self despite documented exclusion semantics"
        )

    return {
        "fixture_sha256": hashlib.sha256(event_text.encode()).hexdigest(),
        "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
        "aircraft_count": len(event.aircraft),
        "own_uav_id": uav_id,
        "own_aircraft_index": aircraft_index,
        "server_includes_own_aircraft": self_included,
        "verified": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("event", type=Path)
    parser.add_argument("metadata", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        result = verify_fixture(args.event, args.metadata)
    except (FixtureVerificationError, OSError, ValueError) as error:
        print(f"AEAC protocol fixture verification failed: {error}")
        return 1
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
