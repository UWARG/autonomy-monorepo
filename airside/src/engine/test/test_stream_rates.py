from __future__ import annotations

import unittest

from engine.constants import MAVLINK_MSG_ID_HEARTBEAT, STREAM_RATE_REQUESTS_HZ
from engine.obstacle_navigation import ObstacleNavigationConfig


class StreamRateTests(unittest.TestCase):
    def test_heartbeat_rate_has_margin_over_telemetry_timeout(self) -> None:
        heartbeat_rate_hz = STREAM_RATE_REQUESTS_HZ[MAVLINK_MSG_ID_HEARTBEAT]
        telemetry_timeout_s = ObstacleNavigationConfig().telemetry_freshness_s

        self.assertGreater(heartbeat_rate_hz * telemetry_timeout_s, 1.0)


if __name__ == "__main__":
    unittest.main()
