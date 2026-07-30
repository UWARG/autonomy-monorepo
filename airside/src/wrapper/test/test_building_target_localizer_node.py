"""ROS message adapter tests."""

import numpy as np
from airside_interfaces.msg import BuildingWing, Plane, ProcessedMap, SpatialTarget
from airside_interfaces.msg import WingBoundary

from building_target_localizer.localizer import BuildingTargetLocalizer
from building_target_localizer.models import LocalizerConfig
from wrapper.building_target_localizer_node import (
    localization_result_to_msg,
    processed_map_from_msg,
)
from utils.src.types import Plane as SharedPlane


def _plane(
    plane_id: str, normal: tuple[float, float, float], offset: float
) -> Plane:
    message = Plane()
    message.id = plane_id
    message.normal.x, message.normal.y, message.normal.z = normal
    message.offset = offset
    message.covariance = [0.0] * 16
    return message


def test_processed_map_round_trip_to_description():
    message = ProcessedMap()
    message.header.frame_id = "mission_frd"
    message.ground_plane_id = "ground"
    message.building_height_m = 6.0
    message.building_height_stddev_m = 0.0
    message.planes = [
        _plane("ground", (0.0, 0.0, 1.0), 0.0),
        _plane("south", (-1.0, 0.0, 0.0), 0.0),
        _plane("west", (0.0, -1.0, 0.0), 0.0),
    ]

    wing = BuildingWing()
    wing.id = "main"
    first = WingBoundary()
    first.plane_id = "south"
    first.opposite_distance_m = 10.0
    second = WingBoundary()
    second.plane_id = "west"
    second.opposite_distance_m = 6.0
    wing.boundaries = [first, second]
    message.wings = [wing]

    target = SpatialTarget()
    target.id = "A"
    target.colour = "BLUE"
    target.position.x = 10.0
    target.position.y = 2.0
    target.position.z = -2.0
    target.covariance = [0.0] * 9
    message.targets = [target]

    snapshot = processed_map_from_msg(message)
    assert snapshot.frame_id == "mission_frd"
    assert isinstance(snapshot.planes[0].plane, SharedPlane)
    assert np.array_equal(snapshot.targets[0].position, [10.0, 2.0, -2.0])

    result = BuildingTargetLocalizer(
        LocalizerConfig(uncertainty_samples=0)
    ).localize(snapshot)
    output = localization_result_to_msg(result)
    assert output.data.startswith("Target A is on")
    assert "The colour is blue." in output.data
