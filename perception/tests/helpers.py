"""Synthetic processed-map fixtures."""

from __future__ import annotations

import numpy as np
from utils.src.types import Plane, Vector3D

from building_target_localizer.models import (
    BuildingWingInput,
    PlaneInput,
    ProcessedMapInput,
    SpatialTargetInput,
    WingBoundaryInput,
)


def plane(
    plane_id: str,
    normal: tuple[float, float, float],
    offset: float,
    covariance: np.ndarray | None = None,
) -> PlaneInput:
    """Create a plane with zero covariance unless one is supplied."""
    return PlaneInput(
        id=plane_id,
        plane=Plane(
            normal=Vector3D(
                x=float(normal[0]),
                y=float(normal[1]),
                z=float(normal[2]),
            ),
            offset=offset,
        ),
        covariance=(
            np.zeros((4, 4), dtype=float)
            if covariance is None
            else np.asarray(covariance, dtype=float)
        ),
    )


def target(
    target_id: str,
    position: tuple[float, float, float],
    colour: str = "BLUE",
    covariance: np.ndarray | None = None,
) -> SpatialTargetInput:
    """Create a synthetic target with zero covariance by default."""
    return SpatialTargetInput(
        id=target_id,
        colour=colour,
        position=np.array(position, dtype=float),
        covariance=(
            np.zeros((3, 3), dtype=float)
            if covariance is None
            else np.asarray(covariance, dtype=float)
        ),
    )


def rectangular_snapshot(
    targets: tuple[SpatialTargetInput, ...] = (),
    length: float = 10.0,
    width: float = 6.0,
    height: float = 6.0,
) -> ProcessedMapInput:
    """Create one axis-aligned rectangular building snapshot."""
    planes = (
        plane("ground", (0.0, 0.0, 1.0), 0.0),
        plane("south_wall", (-1.0, 0.0, 0.0), 0.0),
        plane("west_wall", (0.0, -1.0, 0.0), 0.0),
    )
    wing = BuildingWingInput(
        id="main",
        boundaries=(
            WingBoundaryInput("south_wall", length, 0.0),
            WingBoundaryInput("west_wall", width, 0.0),
        ),
    )
    return ProcessedMapInput(
        frame_id="mission_frd",
        planes=planes,
        ground_plane_id="ground",
        building_height_m=height,
        building_height_stddev_m=0.0,
        wings=(wing,),
        targets=targets,
    )


def l_snapshot(targets: tuple[SpatialTargetInput, ...] = ()) -> ProcessedMapInput:
    """Create two overlapping wings whose union is an L footprint."""
    planes = (
        plane("ground", (0.0, 0.0, 1.0), 0.0),
        plane("a_south", (-1.0, 0.0, 0.0), 0.0),
        plane("a_west", (0.0, -1.0, 0.0), 0.0),
        plane("b_south", (-1.0, 0.0, 0.0), 0.0),
        plane("b_west", (0.0, -1.0, 0.0), 0.0),
    )
    wings = (
        BuildingWingInput(
            id="horizontal",
            boundaries=(
                WingBoundaryInput("a_south", 10.0, 0.0),
                WingBoundaryInput("a_west", 4.0, 0.0),
            ),
        ),
        BuildingWingInput(
            id="vertical",
            boundaries=(
                WingBoundaryInput("b_south", 4.0, 0.0),
                WingBoundaryInput("b_west", 10.0, 0.0),
            ),
        ),
    )
    return ProcessedMapInput(
        frame_id="mission_frd",
        planes=planes,
        ground_plane_id="ground",
        building_height_m=6.0,
        building_height_stddev_m=0.0,
        wings=wings,
        targets=targets,
    )
