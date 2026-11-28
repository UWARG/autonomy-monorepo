"""
Survey input and the checks that run before anything is planned or sent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from gnc.src.orthomosaic import CameraSpec, validate_polygon
from gnc.src.orthomosaic.footprint import validate_camera
from shapely.geometry import Polygon
from utils.src.coordinates import anchor_for_boundary, boundary_to_enu
from utils.src.types import Coordinate

from src.groundside_manager.groundside_manager import DroneState


# kw_only so the many float fields can't be passed in the wrong order.
@dataclass(frozen=True, kw_only=True)
class SurveyRequest:
    """
    One survey: the area to map, the camera, and how complete it must be.
    """

    survey_id: str
    boundary: list[Coordinate]
    camera: CameraSpec
    target_gsd_m: float
    forward_overlap: float = 0.75
    side_overlap: float = 0.65
    # Simulated planned surveys leave float slivers far below 1e-6 m2 (see
    # test_coverage.py). 1 m2 is well above that and is about 10,000 pixels
    # at 1 cm GSD.
    min_gap_area_m2: float = 1.0
    # Ground counts as covered only where a photo sees it at this many times
    # target_gsd_m or finer. 1.5 keeps whole photos up to at least 20 degrees of
    # pitch and stops a steeply tilted photo claiming far-off ground.
    max_gsd_ratio: float = 1.5
    max_reshoot_rounds: int = 2

    @property
    def max_gsd_m(self) -> float:
        """
        Coarsest GSD at which a photo's ground counts as covered.
        """

        return self.max_gsd_ratio * self.target_gsd_m


def validate_request(request: SurveyRequest, drones: list[DroneState]) -> None:
    """
    Raises ``ValueError`` naming the first problem with the survey or fleet.

    Runs before planning so a bad request fails with a clear message instead
    of the planner's bare ``(False, None)``, and before any command is sent.
    """

    if not request.survey_id.strip():
        raise ValueError("survey_id must not be blank")
    _validate_boundary(request.boundary)
    validate_camera(request.camera)
    if not (math.isfinite(request.target_gsd_m) and request.target_gsd_m > 0.0):
        raise ValueError("target_gsd_m must be positive and finite")
    # Same ranges the planner enforces in gnc's spacings().
    if not 0.0 <= request.forward_overlap < 1.0:
        raise ValueError("forward_overlap must be in [0, 1)")
    if not 0.0 <= request.side_overlap < 1.0:
        raise ValueError("side_overlap must be in [0, 1)")
    if not (math.isfinite(request.min_gap_area_m2) and request.min_gap_area_m2 >= 0.0):
        raise ValueError("min_gap_area_m2 must be non-negative and finite")
    # Below 1 even a level photo at the planned altitude would not count.
    if not (math.isfinite(request.max_gsd_ratio) and request.max_gsd_ratio >= 1.0):
        raise ValueError("max_gsd_ratio must be at least 1 and finite")
    # bool is an int subclass; True rounds would be a silent bug.
    if (
        isinstance(request.max_reshoot_rounds, bool)
        or not isinstance(request.max_reshoot_rounds, int)
        or request.max_reshoot_rounds < 0
    ):
        raise ValueError("max_reshoot_rounds must be a non-negative integer")
    _validate_drones(drones)


def _validate_boundary(boundary: list[Coordinate]) -> None:
    if len(boundary) < 3:
        raise ValueError("boundary must have at least 3 points")
    for point in boundary:
        if not (-90.0 <= point.lat <= 90.0 and -180.0 <= point.lon <= 180.0):
            raise ValueError(f"boundary point {point} is not a valid GPS coordinate")
    # Validate in the same local metre frame the planner uses, so a boundary
    # that passes here also passes there.
    anchor = anchor_for_boundary(boundary)
    validate_polygon(Polygon(boundary_to_enu(anchor, boundary)))


def _validate_drones(drones: list[DroneState]) -> None:
    if not drones:
        raise ValueError("at least one drone is required")
    seen: set[str] = set()
    for drone in drones:
        if not drone.drone_id.strip():
            raise ValueError("drone_id must not be blank")
        if drone.drone_id in seen:
            raise ValueError(f"duplicate drone_id {drone.drone_id!r}")
        seen.add(drone.drone_id)
        if not drone.connected:
            raise ValueError(f"drone {drone.drone_id!r} is not connected")
