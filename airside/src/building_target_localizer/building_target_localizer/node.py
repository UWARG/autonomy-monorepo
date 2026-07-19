"""ROS 2 Adapter"""
from __future__ import annotations

import numpy as np
import rclpy
from airside_interfaces.msg import BuildingWing as BuildingWingMsg
from airside_interfaces.msg import LocalizationResult as LocalizationResultMsg
from airside_interfaces.msg import LocalizedTarget as LocalizedTargetMsg
from airside_interfaces.msg import Plane as PlaneMsg
from airside_interfaces.msg import ProcessedMap as ProcessedMapMsg
from airside_interfaces.msg import ReferenceMeasurement as ReferenceMeasurementMsg
from airside_interfaces.msg import SpatialTarget as SpatialTargetMsg
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .localizer import BuildingTargetLocalizer
from .models import (
    BuildingWingInput,
    LocalizerConfig,
    PlaneInput,
    ProcessedMapInput,
    SpatialTargetInput,
    WingBoundaryInput,
)

def _plane_from_msg(message: PlaneMsg) -> PlaneInput:
    return PlaneInput(
        id=message.id,
        normal=np.array(
            [message.normal.x, message.normal.y, message.normal.z], dtype=float
        ),
        offset=float(message.offset),
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

def _point_message(values: np.ndarray) -> Point:
    message = Point()
    message.x = float(values[0])
    message.y = float(values[1])
    message.z = float(values[2])
    return message

def localization_result_to_msg(result, header) -> LocalizationResultMsg:
    """Convert a domain localization batch into its ROS output message."""
    message = LocalizationResultMsg()
    message.header = header
    message.map_valid = result.map_valid
    message.map_error = result.map_error
    for target in result.targets:
        target_message = LocalizedTargetMsg()
        target_message.target_id = target.target_id
        target_message.colour = target.colour
        target_message.status = int(target.status)
        target_message.reason = target.reason
        target_message.source_position = _point_message(target.source_position)
        target_message.snapped_position = _point_message(target.snapped_position)
        target_message.surface_id = target.surface_id
        target_message.surface_type = int(target.surface_type)
        target_message.anchor_id = target.anchor_id
        target_message.anchor_type = int(target.anchor_type)
        target_message.uncertainty_95_m = float(target.uncertainty_95_m)
        target_message.description = target.description
        for measurement in target.measurements:
            measurement_message = ReferenceMeasurementMsg()
            measurement_message.reference_id = measurement.reference_id
            measurement_message.relation = int(measurement.relation)
            measurement_message.distance_m = float(measurement.distance_m)
            measurement_message.uncertainty_95_m = float(
                measurement.uncertainty_95_m
            )
            target_message.measurements.append(measurement_message)
        message.targets.append(target_message)
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
        self._publisher = self.create_publisher(
            LocalizationResultMsg, self.OUTPUT_TOPIC, qos
        )
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
            output = localization_result_to_msg(result, message.header)
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"Processed-map conversion failed: {error}")
            output = LocalizationResultMsg()
            output.header = message.header
            output.map_valid = False
            output.map_error = str(error)
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
