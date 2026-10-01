from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from verify_aeac_protocol_fixture import FixtureVerificationError, verify_fixture


class ProtocolFixtureGateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.event_path = self.root / "captured-event.json"
        self.metadata_path = self.root / "metadata.json"
        self.event_path.write_text(
            json.dumps(
                {
                    "event": "traffic",
                    "payload": {
                        "traffic": [
                            {
                                "aircraftIndex": 9,
                                "name": "SELF",
                                "position": {
                                    "lat": 43.0,
                                    "lon": -80.0,
                                    "altitude": 15.0,
                                    "speed": 0.0,
                                    "direction": 0.0,
                                },
                                "horizontalKeepAway": 5.0,
                                "verticalKeepAway": 3.0,
                            }
                        ]
                    },
                }
            ),
            encoding="utf-8",
        )
        self.metadata = {
            "source": "AEAC test server",
            "sanitized": True,
            "captured_at_utc": "2026-01-01T00:00:00Z",
            "snapshot_semantics": "complete",
            "single_bidirectional_connection_verified": True,
            "server_timestamp_semantics": "absent",
            "heartbeat_semantics": "connection liveness only",
            "disconnect_semantics": "socket close",
            "server_includes_own_aircraft": True,
            "own_aircraft_identity": {"uavId": "WARG-01", "aircraftIndex": 9},
        }

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def write_metadata(self) -> None:
        self.metadata_path.write_text(json.dumps(self.metadata), encoding="utf-8")

    def test_verified_capture_records_hashes_and_identity(self) -> None:
        self.write_metadata()

        result = verify_fixture(self.event_path, self.metadata_path)

        self.assertTrue(result["verified"])
        self.assertEqual(result["own_aircraft_index"], 9)
        self.assertEqual(len(result["fixture_sha256"]), 64)

    def test_synthetic_or_unattested_source_cannot_satisfy_gate(self) -> None:
        self.metadata["source"] = "synthetic"
        self.write_metadata()

        with self.assertRaises(FixtureVerificationError):
            verify_fixture(self.event_path, self.metadata_path)

    def test_identity_inclusion_must_match_capture(self) -> None:
        self.metadata["server_includes_own_aircraft"] = False
        self.write_metadata()

        with self.assertRaises(FixtureVerificationError):
            verify_fixture(self.event_path, self.metadata_path)


if __name__ == "__main__":
    unittest.main()
