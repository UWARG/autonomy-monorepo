"""ROS 2 adapter for building-relative target descriptions."""

from __future__ import annotations

import numpy as np
import rclpy
from airside_interfaces.msg import BuildingWing as BuildingWingMsg
from airside_interfaces.msg import Plane as PlaneMsg
from airside_interfaces.msg import ProcessedMap as ProcessedMapMsg
from airside_interfaces.msg import SpatialTarget as SpatialTargetMsg
from building_target_localizer.localizer import (
    BuildingTargetLocalizer,
    format_descriptions,
)
from building_target_localizer.models import (
    BuildingWingInput,
    LocalizerConfig,
    PlaneInput,
    ProcessedMapInput,
    SpatialTargetInput,
    WingBoundaryInput,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from utils.src.types import Plane, Vector3D


def _plane_from_msg(message: PlaneMsg) -> PlaneInput:
    return PlaneInput(
        id=message.id,
        plane=Plane(
            normal=Vector3D(
                x=float(message.normal.x),
                y=float(message.normal.y),
                z=float(message.normal.z),
            ),
            offset=float(message.offset),
        ),
        covariance=np.asarray(message.covariance, dtype=float).reshape(4, 4),
    )


def _wing_from_msg(message: BuildingWingMsg) -> BuildingWingInput:
    boundaries = tuple(
        WingBoundaryInput(
            plane_id=boundary.plane_id,
            opposite_distance_m=float(boundary.opposite_distance_m),
            distance_stddev_m=float(boundary.distance_stddev_m),
        )
        for boundary in message.boundaries
    )
    if len(boundaries) != 2:
        raise ValueError(f"Wing '{message.id}' must contain exactly two boundaries")
    return BuildingWingInput(
        id=message.id,
        boundaries=(boundaries[0], boundaries[1]),
    )


def _target_from_msg(message: SpatialTargetMsg) -> SpatialTargetInput:
    return SpatialTargetInput(
        id=message.id,
        colour=message.colour,
        position=np.array(
            [message.position.x, message.position.y, message.position.z], dtype=float
        ),
        covariance=np.asarray(message.covariance, dtype=float).reshape(3, 3),
    )


def processed_map_from_msg(message: ProcessedMapMsg) -> ProcessedMapInput:
    """Convert a ROS processed-map snapshot into the domain model."""
    return ProcessedMapInput(
        frame_id=message.header.frame_id,
        planes=tuple(_plane_from_msg(plane) for plane in message.planes),
        ground_plane_id=message.ground_plane_id,
        building_height_m=float(message.building_height_m),
        building_height_stddev_m=float(message.building_height_stddev_m),
        wings=tuple(_wing_from_msg(wing) for wing in message.wings),
        targets=tuple(_target_from_msg(target) for target in message.targets),
    )


def localization_result_to_msg(result) -> String:
    """Convert successful target locations into one plain-text ROS message."""
    message = String()
    message.data = format_descriptions(result)
    return message


class BuildingTargetLocalizerNode(Node):
    """Consume processed maps and publish building-relative target descriptions."""
    INPUT_TOPIC = "/processed_map"
    OUTPUT_TOPIC = "/targets_located"

    def __init__(self) -> None:
        """Create ROS parameters, QoS profiles, publisher, and subscription."""
        super().__init__("building_target_localizer")
        defaults = LocalizerConfig()
        self.declare_parameter("expected_frame_id", defaults.expected_frame_id)
        self.declare_parameter("max_snap_distance_m", defaults.max_snap_distance_m)
        self.declare_parameter(
            "surface_tie_tolerance_m", defaults.surface_tie_tolerance_m
        )
        self.declare_parameter(
            "near_wall_distance_m", defaults.near_wall_distance_m
        )
        self.declare_parameter(
            "anchor_tie_tolerance_m", defaults.anchor_tie_tolerance_m
        )
        self.declare_parameter(
            "wall_vertical_tolerance_deg", defaults.wall_vertical_tolerance_deg
        )
        self.declare_parameter(
            "wing_orthogonality_tolerance_deg",
            defaults.wing_orthogonality_tolerance_deg,
        )
        self.declare_parameter(
            "wing_join_tolerance_m", defaults.wing_join_tolerance_m
        )
        self.declare_parameter("condition_epsilon", defaults.condition_epsilon)
        self.declare_parameter("uncertainty_samples", defaults.uncertainty_samples)
        self.declare_parameter("uncertainty_seed", defaults.uncertainty_seed)
        self.declare_parameter(
            "max_unstable_sample_fraction",
            defaults.max_unstable_sample_fraction,
        )

        config = LocalizerConfig(
            expected_frame_id=str(self.get_parameter("expected_frame_id").value),
            max_snap_distance_m=float(
                self.get_parameter("max_snap_distance_m").value
            ),
            surface_tie_tolerance_m=float(
                self.get_parameter("surface_tie_tolerance_m").value
            ),
            near_wall_distance_m=float(
                self.get_parameter("near_wall_distance_m").value
            ),
            anchor_tie_tolerance_m=float(
                self.get_parameter("anchor_tie_tolerance_m").value
            ),
            wall_vertical_tolerance_deg=float(
                self.get_parameter("wall_vertical_tolerance_deg").value
            ),
            wing_orthogonality_tolerance_deg=float(
                self.get_parameter("wing_orthogonality_tolerance_deg").value
            ),
            wing_join_tolerance_m=float(
                self.get_parameter("wing_join_tolerance_m").value
            ),
            condition_epsilon=float(
                self.get_parameter("condition_epsilon").value
            ),
            uncertainty_samples=int(
                self.get_parameter("uncertainty_samples").value
            ),
            uncertainty_seed=int(self.get_parameter("uncertainty_seed").value),
            max_unstable_sample_fraction=float(
                self.get_parameter("max_unstable_sample_fraction").value
            ),
        )
        self._localizer = BuildingTargetLocalizer(config)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, self.OUTPUT_TOPIC, qos)
        self._subscription = self.create_subscription(
            ProcessedMapMsg, self.INPUT_TOPIC, self._on_processed_map, qos
        )
        self.get_logger().info(
            f"Building target localizer ready: '{self.INPUT_TOPIC}' -> "
            f"'{self.OUTPUT_TOPIC}' in frame '{config.expected_frame_id}'."
        )

    def _on_processed_map(self, message: ProcessedMapMsg) -> None:
        try:
            snapshot = processed_map_from_msg(message)
            result = self._localizer.localize(snapshot)
            output = localization_result_to_msg(result)
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"Processed-map conversion failed: {error}")
            return
        if not result.map_valid:
            self.get_logger().error(f"Processed map is invalid: {result.map_error}")
        for target in result.targets:
            if not target.description:
                self.get_logger().warning(
                    f"Target '{target.target_id}' was not described: {target.reason}"
                )
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    """Run the building target localizer node until ROS shuts down."""
    rclpy.init(args=args)
    node = BuildingTargetLocalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
