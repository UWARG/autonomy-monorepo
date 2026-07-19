from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

class LocalizationStatus(IntEnum):
    """Per-target localization outcome."""
    OK = 0
    INVALID_TARGET = 1
    MAP_INVALID = 2
    NO_SURFACE = 3
    AMBIGUOUS_SURFACE = 4
    AMBIGUOUS_ANCHOR = 5
    UNSTABLE_GEOMETRY = 6


class SurfaceType(IntEnum):
    """Surface selected for a target."""
    NONE = 0
    WALL = 1
    GROUND = 2


class AnchorType(IntEnum):
    """Building feature used to anchor a description."""
    NONE = 0
    OUTER_CORNER = 1
    INNER_CORNER = 2
    TWO_CORNERS = 3


class Relation(IntEnum):
    """Machine-readable spatial relationship."""
    DISTANCE = 0
    NORTH_OF = 1
    NORTHEAST_OF = 2
    EAST_OF = 3
    SOUTHEAST_OF = 4
    SOUTH_OF = 5
    SOUTHWEST_OF = 6
    WEST_OF = 7
    NORTHWEST_OF = 8
    ABOVE = 9
    BELOW = 10
    OUT_FROM = 11


@dataclass(frozen=True)
class PlaneInput:
    """Plane equation and uncertainty in mission FRD coordinates."""
    id: str
    normal: np.ndarray
    offset: float
    covariance: np.ndarray


@dataclass(frozen=True)
class WingBoundaryInput:
    """Observed wing wall and distance to its opposing wall."""
    plane_id: str
    opposite_distance_m: float
    distance_stddev_m: float


@dataclass(frozen=True)
class BuildingWingInput:
    """Rectangular wing described by two adjacent observed walls."""
    id: str
    boundaries: tuple[WingBoundaryInput, WingBoundaryInput]


@dataclass(frozen=True)
class SpatialTargetInput:
    """Target point and uncertainty in mission FRD coordinates."""
    id: str
    colour: str
    position: np.ndarray
    covariance: np.ndarray


@dataclass(frozen=True)
class ProcessedMapInput:
    """Complete localization input snapshot."""
    frame_id: str
    planes: tuple[PlaneInput, ...]
    ground_plane_id: str
    building_height_m: float
    building_height_stddev_m: float
    wings: tuple[BuildingWingInput, ...]
    targets: tuple[SpatialTargetInput, ...]


@dataclass(frozen=True)
class LocalizerConfig:
    """Validation and selection thresholds exposed as ROS parameters."""
    expected_frame_id: str = "mission_frd"
    max_snap_distance_m: float = 0.5
    surface_tie_tolerance_m: float = 0.1
    near_wall_distance_m: float = 5.0
    anchor_tie_tolerance_m: float = 0.25
    wall_vertical_tolerance_deg: float = 5.0
    wing_orthogonality_tolerance_deg: float = 5.0
    wing_join_tolerance_m: float = 0.05
    condition_epsilon: float = 1.0e-6
    uncertainty_samples: int = 1000
    uncertainty_seed: int = 97
    max_unstable_sample_fraction: float = 0.05


@dataclass
class ReferenceMeasurementResult:
    """One distance and its semantic relation to a reference feature."""
    reference_id: str
    relation: Relation
    distance_m: float
    uncertainty_95_m: float = 0.0


@dataclass
class LocalizedTargetResult:
    """Internal representation of a target result."""
    target_id: str
    colour: str
    status: LocalizationStatus
    reason: str
    source_position: np.ndarray
    snapped_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    surface_id: str = ""
    surface_type: SurfaceType = SurfaceType.NONE
    surface_name: str = ""
    anchor_id: str = ""
    anchor_type: AnchorType = AnchorType.NONE
    reference_names: tuple[str, ...] = ()
    measurements: list[ReferenceMeasurementResult] = field(default_factory=list)
    uncertainty_95_m: float = 0.0
    description: str = ""
    description_body: str = ""


@dataclass
class LocalizationBatchResult:
    """Internal result for one processed-map snapshot."""
    map_valid: bool
    map_error: str
    targets: list[LocalizedTargetResult]


@dataclass(frozen=True)
class CornerFeature:
    """Ground-level exterior or reflex corner."""
    id: str
    point_2d: np.ndarray
    point_3d: np.ndarray
    reflex: bool
    wing_ids: tuple[str, ...]
    name: str


@dataclass(frozen=True)
class WallFeature:
    """Finite exterior wall reconstructed from the merged footprint."""
    id: str
    start_2d: np.ndarray
    end_2d: np.ndarray
    start_3d: np.ndarray
    end_3d: np.ndarray
    outward_3d: np.ndarray
    wing_ids: tuple[str, ...]
    name: str
    start_corner_id: str
    end_corner_id: str


@dataclass(frozen=True)
class BuildingModel:
    """Merged finite building geometry used by the localizer."""
    ground_plane: PlaneInput
    height_m: float
    footprint: object
    wing_footprints: dict[str, object]
    corners: tuple[CornerFeature, ...]
    walls: tuple[WallFeature, ...]

    def ground_point(self, point_2d: np.ndarray) -> np.ndarray:
        """Lift an x/y point onto the fitted ground plane."""
        normal = self.ground_plane.normal
        z = -(
            normal[0] * point_2d[0]
            + normal[1] * point_2d[1]
            + self.ground_plane.offset
        ) / normal[2]
        return np.array([point_2d[0], point_2d[1], z], dtype=float)

    def roof_point(self, point_2d: np.ndarray) -> np.ndarray:
        """Lift an x/y point onto the roof along mission up."""
        return self.ground_point(point_2d) - self.height_m * self.ground_plane.normal
