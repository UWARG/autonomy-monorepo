from __future__ import annotations

import math
import unittest

from synthetic_laserscan import (
    crossed_wall_segment,
    point_to_wall_distance_m,
    wall_scan_ranges,
)


class SyntheticLaserScanTests(unittest.TestCase):
    def test_ros_flu_north_beam_hits_wall_when_yaw_is_east(self) -> None:
        ranges = wall_scan_ranges(
            east_m=0.0,
            north_m=0.0,
            yaw_enu_rad=0.0,
            wall_north_m=20.0,
            wall_half_width_m=6.0,
            angle_min_rad=-math.pi,
            angle_increment_rad=math.pi / 2.0,
            beam_count=4,
            range_max_m=50.0,
        )
        self.assertTrue(math.isinf(ranges[2]))
        self.assertAlmostEqual(ranges[3], 20.0)

    def test_yaw_rotates_body_forward_into_world_north(self) -> None:
        ranges = wall_scan_ranges(
            east_m=0.0,
            north_m=5.0,
            yaw_enu_rad=math.pi / 2.0,
            wall_north_m=20.0,
            wall_half_width_m=6.0,
            angle_min_rad=0.0,
            angle_increment_rad=math.pi / 2.0,
            beam_count=4,
            range_max_m=50.0,
        )
        self.assertAlmostEqual(ranges[0], 15.0)

    def test_finite_wall_distance_and_crossing(self) -> None:
        self.assertAlmostEqual(
            point_to_wall_distance_m(
                8.0,
                18.0,
                wall_north_m=20.0,
                wall_half_width_m=6.0,
            ),
            math.sqrt(8.0),
        )
        self.assertTrue(
            crossed_wall_segment(
                (0.0, 19.0),
                (0.0, 21.0),
                wall_north_m=20.0,
                wall_half_width_m=6.0,
            )
        )
        self.assertFalse(
            crossed_wall_segment(
                (7.0, 19.0),
                (7.0, 21.0),
                wall_north_m=20.0,
                wall_half_width_m=6.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
