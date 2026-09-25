from __future__ import annotations

import unittest

from unknown_sector_probe import run_probe


class UnknownSectorProbeTests(unittest.TestCase):
    def test_all_none_sectors_record_current_physical_flight_blocker(self) -> None:
        result = run_probe()

        self.assertEqual(result["input_sector_count"], 72)
        self.assertEqual(result["input_none_count"], 72)
        self.assertTrue(result["snapshot_healthy"])
        self.assertEqual(result["snapshot_obstacle_count"], 0)
        self.assertEqual(result["planner_status"], "PATH_FOUND")
        self.assertTrue(result["physical_flight_blocker"])


if __name__ == "__main__":
    unittest.main()
