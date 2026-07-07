from __future__ import annotations

import py_trees
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from engine import blackboard_keys
from airside.src.engine.engine.utils.waypoint_parser import parse_waypoints_file, sort_clockwise_sweep

WAYPOINTS_FILE_PARAMETER = "waypoints_file"


class LoadWaypointList(py_trees.behaviour.Behaviour):
    """
    Loads the lap waypoints from a text file onto the blackboard.

    Parses the waypoints file (``lat, lon, alt`` waypoint per line, with
    an optional ``h`` home line) and orders the waypoints as a clockwise
    sweep around their centroid, starting from the waypoint nearest home's
    direction off the centroid (or north if no home is marked). Writes the
    resulting list to ``waypoints``. Returns SUCCESS once loaded, FAILURE
    if the file is missing, malformed, or has no lap waypoints.
    """

    def __init__(self, name: str = "LoadWaypointList") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.WAYPOINTS, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

        default_path = (
            f"{get_package_share_directory('engine')}/config/waypoints.txt"
        )
        if not self._node.has_parameter(WAYPOINTS_FILE_PARAMETER):
            self._node.declare_parameter(WAYPOINTS_FILE_PARAMETER, default_path)

    def update(self) -> py_trees.common.Status:
        waypoints_file = (
            self._node.get_parameter(WAYPOINTS_FILE_PARAMETER)
            .get_parameter_value()
            .string_value
        )

        try:
            home, lap_waypoints = parse_waypoints_file(waypoints_file)
        except (OSError, ValueError) as error:
            self._node.get_logger().error(
                f"{self.name}: failed to load '{waypoints_file}': {error}"
            )
            return py_trees.common.Status.FAILURE

        if not lap_waypoints:
            self._node.get_logger().error(
                f"{self.name}: no lap waypoints found in '{waypoints_file}'"
            )
            return py_trees.common.Status.FAILURE

        waypoints = sort_clockwise_sweep(lap_waypoints, home=home)

        self.blackboard.set(blackboard_keys.WAYPOINTS, waypoints)
        self._node.get_logger().info(
            f"{self.name}: loaded {len(waypoints)} waypoints from "
            f"'{waypoints_file}' (home: {home}): {waypoints}"
        )
        return py_trees.common.Status.SUCCESS
