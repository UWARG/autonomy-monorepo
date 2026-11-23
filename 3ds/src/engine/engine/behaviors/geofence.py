"""
Loads the geofence from YAML and uploads it to ArduPilot via MAVROS.
"""

from __future__ import annotations

import py_trees
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush

from engine import blackboard_keys
from engine.fence import Fence
from utils.src.waypoint_utils import parse_geofence_file

_GEOFENCE_FILE_PARAMETER = "geofence_file"


class LoadGeofence(py_trees.behaviour.Behaviour):
    """
    Loads the geofence from a YAML file onto the blackboard.

    The default path is the config installed with this package; set the
    ``geofence_file`` ROS parameter to load a different polygon. Returns
    SUCCESS once loaded, FAILURE if the file is missing or malformed.
    """

    def __init__(self, name: str = "LoadGeofence") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.GEOFENCE, access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

        default_path = (
            f"{get_package_share_directory('engine')}/config/geofence.yaml"
        )
        if not self._node.has_parameter(_GEOFENCE_FILE_PARAMETER):
            self._node.declare_parameter(_GEOFENCE_FILE_PARAMETER, default_path)

    def update(self) -> py_trees.common.Status:
        path = (
            self._node.get_parameter(_GEOFENCE_FILE_PARAMETER)
            .get_parameter_value()
            .string_value
        )

        try:
            fence = Fence(parse_geofence_file(path))
        except (OSError, ValueError) as error:
            self._node.get_logger().error(
                f"{self.name}: failed to load '{path}': {error}"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.set(blackboard_keys.GEOFENCE, fence)
        self._node.get_logger().info(
            f"{self.name}: loaded {len(fence.vertices)} vertices from '{path}'"
        )
        return py_trees.common.Status.SUCCESS


class Geofence(py_trees.behaviour.Behaviour):
    """Uploads the configured fence polygon to ArduPilot via MAVROS."""

    def __init__(self, name: str = "Geofence") -> None:
        super().__init__(name=name)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=blackboard_keys.GEOFENCE, access=py_trees.common.Access.READ
        )

    @staticmethod
    def _create_polygon(vertices):
        if len(vertices) < 3:
            raise ValueError(
                "geofence: at least 3 vertices is needed to form a valid polygon"
            )

        waypoints = []
        for lat, lon in vertices:
            waypoint = Waypoint()
            waypoint.frame = Waypoint.FRAME_GLOBAL
            waypoint.command = 5001
            waypoint.is_current = False
            waypoint.autocontinue = True
            waypoint.param1 = float(len(vertices))
            waypoint.param2 = 0.0
            waypoint.x_lat = float(lat)
            waypoint.y_long = float(lon)
            waypoint.z_alt = 0.0
            waypoints.append(waypoint)
        return waypoints

    def upload_geofence(self, vertices):
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = self._create_polygon(vertices)
        return self.push_client.call_async(request)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        """
        Called once during tree.setup().
        """

        self._node = kwargs["node"]
        self.push_client = self._node.create_client(
            WaypointPush, "/mavros/geofence/push"
        )

    def initialise(self) -> None:
        """
        Called each time this behavior transitions from IDLE to RUNNING.
        """
        self.upload_future = None

    def update(self) -> py_trees.common.Status:
        """
        Called on every tick while RUNNING.
        """
        try:
            fence = self.blackboard.get(blackboard_keys.GEOFENCE)
        except KeyError:
            self._node.get_logger().error(f"{self.name}: no geofence loaded")
            return py_trees.common.Status.FAILURE

        if not self.push_client.service_is_ready():
            return py_trees.common.Status.RUNNING

        if self.upload_future is None:
            self.upload_future = self.upload_geofence(
                [(vertex.lat, vertex.lon) for vertex in fence.vertices]
            )
            return py_trees.common.Status.RUNNING

        if not self.upload_future.done():
            return py_trees.common.Status.RUNNING

        response = self.upload_future.result()
        if response and response.success:
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().error("Failed to upload geofence waypoints")
        return py_trees.common.Status.FAILURE
