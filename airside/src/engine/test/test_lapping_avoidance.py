from __future__ import annotations

import unittest

from engine.behaviors.navigation.fly_to_waypoint import FlyToWaypoint
from engine.behaviors.navigation.obstacle_aware_fly_to_waypoint import (
    ObstacleAwareFlyToWaypoint,
)
from engine.subtrees.lapping import create_lapping_subtree


class LappingTreeTests(unittest.TestCase):
    def test_lapping_uses_only_obstacle_aware_waypoint_navigation(self) -> None:
        tree = create_lapping_subtree()
        behaviours = list(tree.iterate())
        self.assertEqual(
            sum(isinstance(item, ObstacleAwareFlyToWaypoint) for item in behaviours),
            1,
        )
        self.assertFalse(any(isinstance(item, FlyToWaypoint) for item in behaviours))


if __name__ == "__main__":
    unittest.main()
