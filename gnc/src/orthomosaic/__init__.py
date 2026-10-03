"""Orthomosaic lawn-mower coverage planning."""

from .coordinates import (
    anchor_for_boundary,
    boundary_to_enu,
    coordinate_to_enu,
    enu_to_coordinate,
    rotate_point,
    rotate_points,
    rotate_polygon,
)
from .decompose import (
    HORIZONTAL,
    VERTICAL,
    Cell,
    decompose_polygon,
    is_monotone,
    order_cells,
    validate_polygon,
)
from .footprint import (
    derive_altitude,
    ground_footprint,
    ground_sample_distance,
    spacings,
)
from .models import CameraSpec, ScanPlan, ScanRequest
from .pathing import plan_orthomosaic_path
from .sweep import (
    SweepParams,
    SweepResult,
    coverage_cost,
    default_turn_penalty_m,
    path_length,
    sweep_cell,
)

__all__ = [
    "CameraSpec",
    "Cell",
    "HORIZONTAL",
    "ScanPlan",
    "ScanRequest",
    "SweepParams",
    "SweepResult",
    "VERTICAL",
    "anchor_for_boundary",
    "boundary_to_enu",
    "coordinate_to_enu",
    "coverage_cost",
    "decompose_polygon",
    "default_turn_penalty_m",
    "derive_altitude",
    "enu_to_coordinate",
    "ground_footprint",
    "ground_sample_distance",
    "is_monotone",
    "order_cells",
    "path_length",
    "plan_orthomosaic_path",
    "rotate_point",
    "rotate_points",
    "rotate_polygon",
    "spacings",
    "sweep_cell",
    "validate_polygon",
]