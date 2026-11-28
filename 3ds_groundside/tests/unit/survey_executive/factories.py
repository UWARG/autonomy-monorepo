"""
Shared builders for survey executive unit tests.
"""

from __future__ import annotations

import math

from gnc.src.orthomosaic import CameraSpec
from shapely.geometry import Polygon
from utils.src.coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
)
from utils.src.types import Coordinate

from src.drone_task_scheduler.mission import Mission
from src.groundside_manager.groundside_manager import DroneState, MissionState
from src.survey_executive import SurveyRequest, photo_footprint

# Roughly a 110 m x 80 m rectangle near Waterloo.
LAT0, LON0 = 43.4723, -80.5449
RECTANGLE = [
    Coordinate(LAT0, LON0, 0.0),
    Coordinate(LAT0, LON0 + 0.001, 0.0),
    Coordinate(LAT0 + 0.001, LON0 + 0.001, 0.0),
    Coordinate(LAT0 + 0.001, LON0, 0.0),
]
CAMERA = CameraSpec(
    image_width_px=4000,
    image_height_px=3000,
    horizontal_fov_rad=math.radians(70.0),
    vertical_fov_rad=math.radians(55.0),
)


def make_request(**overrides: object) -> SurveyRequest:
    fields: dict[str, object] = {
        "survey_id": "survey-1",
        "boundary": RECTANGLE,
        "camera": CAMERA,
        # 1 cm GSD flies at about 29 m with a 40 m x 30 m footprint, so the
        # rectangle needs several passes.
        "target_gsd_m": 0.01,
    }
    fields.update(overrides)
    return SurveyRequest(**fields)  # type: ignore[arg-type]


def make_drone(
    drone_id: str = "drone-01",
    connected: bool = True,
    position: Coordinate | None = None,
) -> DroneState:
    return DroneState(
        drone_id=drone_id,
        vehicle_type="QUADCOPTER",
        position=position,
        orientation=None,
        mission_state=MissionState.IDLE,
        active_task_id=None,
        last_update_timestamp=0.0,
        connected=connected,
    )


def boundary_enu(boundary: list[Coordinate]) -> Polygon:
    anchor = anchor_for_boundary(boundary)
    return Polygon(boundary_to_enu(anchor, boundary))


def nadir_photos(
    request: SurveyRequest, routes: dict[str, list[Mission]]
) -> dict[str, Polygon]:
    """
    Level photo footprints taken exactly at every planned setpoint.
    """

    anchor = anchor_for_boundary(request.boundary)
    photos: dict[str, Polygon] = {}
    for missions in routes.values():
        for mission in missions:
            east_m, north_m = coordinate_to_enu(anchor, mission.target)
            footprint = photo_footprint(
                east_m,
                north_m,
                mission.target.alt,
                0.0,
                0.0,
                math.degrees(mission.yaw),
                request.camera,
                max_gsd_m=request.max_gsd_m,
            )
            assert footprint is not None
            photos[mission.task_id] = footprint
    return photos
