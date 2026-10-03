"""Data model for orthomosaic scan planning."""

from __future__ import annotations

from dataclasses import dataclass

from utils.src.types import Coordinate


@dataclass(frozen=True)
class CameraSpec:
    """Camera parameters needed to compute ground coverage."""

    image_width_px: int
    image_height_px: int
    horizontal_fov_rad: float
    vertical_fov_rad: float


@dataclass(frozen=True)
class ScanRequest:
    """Everything the planner needs to produce an orthomosaic scan plan."""

    boundary: list[Coordinate]
    camera: CameraSpec
    target_gsd_m: float
    forward_overlap: float = 0.75
    side_overlap: float = 0.65
    start: Coordinate | None = None
    turn_penalty_m: float | None = None
    angle_step_deg: float = 5.0


@dataclass(frozen=True)
class ScanPlan:
    """The planned coverage: GPS waypoints plus diagnostic metadata."""

    capture_waypoints: list[Coordinate]
    path_waypoints: list[Coordinate]
    altitude_agl_m: float
    achieved_gsd_m: float
    footprint_width_m: float
    footprint_length_m: float
    line_spacing_m: float
    capture_spacing_m: float
    sweep_angle_deg: float
    cell_count: int
    estimated_path_length_m: float
    estimated_turn_count: int
    photo_count: int