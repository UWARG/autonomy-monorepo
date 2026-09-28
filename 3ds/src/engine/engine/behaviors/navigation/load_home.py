from __future__ import annotations

import py_trees
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from engine import blackboard_keys
from utils.src.waypoint_utils import parse_waypoints_file

_HOME_FILE_PARAMETER = "home_file"


class LoadHome(py_trees.behaviour.Behaviour):
    """
    Loads the home coordinate from a YAML config file onto the blackboard.

    Writes the ``home`` entry of the file to ``home_waypoint``. Returns
    SUCCESS once loaded, FAILURE if the file is missing, malformed, or has
    no home coordinate.
    """

    def __init__(self, name: str = "LoadHome") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.HOME_WAYPOINT, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

        default_path = f"{get_package_share_directory('engine')}/config/home.yaml"
        if not self._node.has_parameter(_HOME_FILE_PARAMETER):
            self._node.declare_parameter(_HOME_FILE_PARAMETER, default_path)

    def update(self) -> py_trees.common.Status:
        home_file = (
            self._node.get_parameter(_HOME_FILE_PARAMETER)
            .get_parameter_value()
            .string_value
        )

        try:
            home, _ = parse_waypoints_file(home_file)
        except (OSError, ValueError) as error:
            self._node.get_logger().error(
                f"{self.name}: failed to load '{home_file}': {error}"
            )
            return py_trees.common.Status.FAILURE

        if home is None:
            self._node.get_logger().error(
                f"{self.name}: no home coordinate found in '{home_file}'"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.HOME_WAYPOINT, home)
        self._node.get_logger().info(
            f"{self.name}: loaded home {home} from '{home_file}'"
        )
        return py_trees.common.Status.SUCCESS
