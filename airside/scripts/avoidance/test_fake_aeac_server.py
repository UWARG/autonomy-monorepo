from __future__ import annotations

import math
import unittest

from fake_aeac_server import FakeTrafficConfig, offset_coordinate, traffic_payload

TELEMETRY = {
    "uavId": "WARG-01",
    "latitude": 43.0,
    "longitude": -80.0,
    "altitudeAGL": 15.0,
}


class FakeAeacPayloadTests(unittest.TestCase):
    def test_clear_snapshot_is_a_fresh_valid_empty_list(self) -> None:
        payload = traffic_payload(
            config=FakeTrafficConfig("clear"),
            own_telemetry=TELEMETRY,
            elapsed_s=0.0,
        )

        self.assertEqual(payload["payload"]["traffic"], [])

    def test_static_intruder_is_twenty_metres_north(self) -> None:
        payload = traffic_payload(
            config=FakeTrafficConfig("static"),
            own_telemetry=TELEMETRY,
            elapsed_s=0.0,
        )
        intruder = payload["payload"]["traffic"][1]
        north_m = math.radians(
            intruder["position"]["lat"] - TELEMETRY["latitude"]
        ) * 6_371_000.0

        self.assertAlmostEqual(north_m, 20.0, places=3)
        self.assertEqual(intruder["position"]["speed"], 0.0)
        self.assertEqual(intruder["position"]["altitude"], 15.0)

    def test_crossing_intruder_position_and_reported_velocity_agree(self) -> None:
        start = traffic_payload(
            config=FakeTrafficConfig("crossing"),
            own_telemetry=TELEMETRY,
            elapsed_s=0.0,
            crossing_elapsed_s=0.0,
        )["payload"]["traffic"][1]
        later = traffic_payload(
            config=FakeTrafficConfig("crossing"),
            own_telemetry=TELEMETRY,
            elapsed_s=2.0,
            crossing_elapsed_s=2.0,
        )["payload"]["traffic"][1]

        self.assertEqual(start["position"]["speed"], 4.0)
        self.assertEqual(start["position"]["direction"], 90.0)
        self.assertGreater(later["position"]["lon"], start["position"]["lon"])

    def test_coordinate_offset_preserves_zero_and_signs(self) -> None:
        unchanged = offset_coordinate(43.0, -80.0, 0.0, 0.0)
        northeast = offset_coordinate(43.0, -80.0, 10.0, 10.0)

        self.assertEqual(unchanged, (43.0, -80.0))
        self.assertGreater(northeast[0], 43.0)
        self.assertGreater(northeast[1], -80.0)


if __name__ == "__main__":
    unittest.main()
