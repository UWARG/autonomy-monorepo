"""Building reconstruction and finite-surface geometry."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
from shapely.geometry import MultiPolygon, Point, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from utils.src.types import Plane, Vector3D

from .models import (
    BuildingModel,
    CornerFeature,
    LocalizerConfig,
    PlaneInput,
    ProcessedMapInput,
    WallFeature,
)

class BuildingGeometryError(ValueError):
    """Raised when a processed map cannot define one trustworthy building."""

_COMPASS_NAMES = (
    "north",
    "northeast",
    "east",
    "southeast",
    "south",
    "southwest",
    "west",
    "northwest",
)

def compass_name(vector_2d: np.ndarray) -> str:
    """Return the nearest eight-way compass sector in mission FRD x/y."""
    vector = np.asarray(vector_2d, dtype=float)
    if vector.shape != (2,) or not np.all(np.isfinite(vector)):
        raise ValueError("Compass vector must contain two finite values")
    if np.linalg.norm(vector) <= 1.0e-12:
        return "at"
    angle = math.atan2(float(vector[1]), float(vector[0]))
    sector = int(math.floor(angle / (math.pi / 4.0) + 0.5)) % 8
    return _COMPASS_NAMES[sector]

def validate_covariance(covariance: np.ndarray, size: int, label: str) -> np.ndarray:
    """Validate and symmetrize a finite positive-semidefinite covariance."""
    value = np.asarray(covariance, dtype=float)
    if value.shape != (size, size):
        raise ValueError(f"{label} covariance must be {size}x{size}")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{label} covariance contains non-finite values")
    if not np.allclose(value, value.T, atol=1.0e-8):
        raise ValueError(f"{label} covariance must be symmetric")
    value = (value + value.T) / 2.0
    if float(np.min(np.linalg.eigvalsh(value))) < -1.0e-9:
        raise ValueError(f"{label} covariance must be positive semidefinite")
    return value

def normalize_plane(plane: PlaneInput, epsilon: float) -> PlaneInput:
    """Return a normalized plane after validating its values and covariance."""
    if not plane.id:
        raise BuildingGeometryError("Every plane must have a non-empty id")
    normal = np.asarray(plane.normal, dtype=float)
    if normal.shape != (3,) or not np.all(np.isfinite(normal)):
        raise BuildingGeometryError(f"Plane '{plane.id}' has an invalid normal")
    if not math.isfinite(plane.offset):
        raise BuildingGeometryError(f"Plane '{plane.id}' has an invalid offset")
    norm = float(np.linalg.norm(normal))
    if norm <= epsilon:
        raise BuildingGeometryError(f"Plane '{plane.id}' has a zero normal")
    try:
        covariance = validate_covariance(
            plane.covariance, 4, f"Plane '{plane.id}'"
        )
    except ValueError as error:
        raise BuildingGeometryError(str(error)) from error
    unit_normal = normal / norm
    normalized_offset = float(plane.offset) / norm
    jacobian = np.zeros((4, 4), dtype=float)
    jacobian[:3, :3] = (
        np.eye(3, dtype=float) - np.outer(unit_normal, unit_normal)
    ) / norm
    jacobian[3, :3] = -normalized_offset * unit_normal / norm
    jacobian[3, 3] = 1.0 / norm
    normalized_covariance = jacobian @ covariance @ jacobian.T
    return replace(
        plane,
        plane=Plane(
            normal=Vector3D(
                x=float(unit_normal[0]),
                y=float(unit_normal[1]),
                z=float(unit_normal[2]),
            ),
            offset=normalized_offset,
        ),
        covariance=(normalized_covariance + normalized_covariance.T) / 2.0,
    )

def _wall_line(plane: PlaneInput, ground: PlaneInput) -> tuple[np.ndarray, float, float]:
    """Project a vertical plane to an x/y line at the ground origin."""
    projected_normal = np.array(
        [
            plane.normal[0]
            - plane.normal[2] * ground.normal[0] / ground.normal[2],
            plane.normal[1]
            - plane.normal[2] * ground.normal[1] / ground.normal[2],
        ],
        dtype=float,
    )
    projected_norm = float(np.linalg.norm(projected_normal))
    if projected_norm <= 1.0e-9:
        raise BuildingGeometryError(f"Wall plane '{plane.id}' has no horizontal normal")
    line_normal = projected_normal / projected_norm
    line_offset = (
        plane.offset
        - plane.normal[2] * ground.offset / ground.normal[2]
    ) / projected_norm
    return line_normal, float(line_offset), projected_norm

def _intersect_lines(
    first_normal: np.ndarray,
    first_offset: float,
    first_level: float,
    second_normal: np.ndarray,
    second_offset: float,
    second_level: float,
    epsilon: float,
) -> np.ndarray:
    matrix = np.vstack((first_normal, second_normal))
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) <= epsilon:
        raise BuildingGeometryError("Wing reference walls are parallel or ill-conditioned")
    right_hand_side = np.array(
        [first_level - first_offset, second_level - second_offset], dtype=float
    )
    return np.linalg.solve(matrix, right_hand_side)

def _wing_polygon(
    snapshot: ProcessedMapInput,
    plane_by_id: dict[str, PlaneInput],
    ground: PlaneInput,
    wing_index: int,
    config: LocalizerConfig,
) -> Polygon:
    wing = snapshot.wings[wing_index]
    if not wing.id:
        raise BuildingGeometryError("Every wing must have a non-empty id")
    if len(wing.boundaries) != 2:
        raise BuildingGeometryError(f"Wing '{wing.id}' must have exactly two boundaries")
    first_boundary, second_boundary = wing.boundaries
    if first_boundary.plane_id == second_boundary.plane_id:
        raise BuildingGeometryError(
            f"Wing '{wing.id}' must reference two different wall planes"
        )
    try:
        first_plane = plane_by_id[first_boundary.plane_id]
        second_plane = plane_by_id[second_boundary.plane_id]
    except KeyError as error:
        raise BuildingGeometryError(
            f"Wing '{wing.id}' references unknown plane '{error.args[0]}'"
        ) from error

    for boundary in wing.boundaries:
        if (
            not math.isfinite(boundary.opposite_distance_m)
            or boundary.opposite_distance_m <= 0.0
        ):
            raise BuildingGeometryError(
                f"Wing '{wing.id}' has a non-positive opposing-wall distance"
            )
        if (
            not math.isfinite(boundary.distance_stddev_m)
            or boundary.distance_stddev_m < 0.0
        ):
            raise BuildingGeometryError(
                f"Wing '{wing.id}' has an invalid distance standard deviation"
            )

    vertical_limit = math.sin(math.radians(config.wall_vertical_tolerance_deg))
    for plane in (first_plane, second_plane):
        if abs(float(np.dot(plane.normal, ground.normal))) > vertical_limit:
            raise BuildingGeometryError(
                f"Wing '{wing.id}' plane '{plane.id}' is not vertical to the ground"
            )

    orthogonality_limit = math.sin(
        math.radians(config.wing_orthogonality_tolerance_deg)
    )
    if abs(float(np.dot(first_plane.normal, second_plane.normal))) > orthogonality_limit:
        raise BuildingGeometryError(
            f"Wing '{wing.id}' reference walls are not near-orthogonal"
        )

    first_normal, first_offset, first_scale = _wall_line(first_plane, ground)
    second_normal, second_offset, second_scale = _wall_line(second_plane, ground)
    first_span = first_boundary.opposite_distance_m / first_scale
    second_span = second_boundary.opposite_distance_m / second_scale

    points = [
        _intersect_lines(
            first_normal,
            first_offset,
            first_level,
            second_normal,
            second_offset,
            second_level,
            config.condition_epsilon,
        )
        for first_level, second_level in (
            (0.0, 0.0),
            (-first_span, 0.0),
            (-first_span, -second_span),
            (0.0, -second_span),
        )
    ]
    polygon = Polygon(points)
    if not polygon.is_valid or polygon.area <= config.condition_epsilon:
        raise BuildingGeometryError(f"Wing '{wing.id}' produced an invalid footprint")
    return orient(polygon, sign=1.0)

def _canonical_ring(polygon: Polygon) -> list[np.ndarray]:
    """Return a CCW exterior ring with a stable lexicographic first vertex."""
    coordinates = [np.array(value, dtype=float) for value in orient(
        polygon, sign=1.0
    ).exterior.coords[:-1]]
    coordinates = [
        point
        for index, point in enumerate(coordinates)
        if np.linalg.norm(point - coordinates[index - 1]) > 1.0e-9
    ]
    changed = True
    while changed and len(coordinates) > 3:
        changed = False
        simplified = []
        for index, point in enumerate(coordinates):
            previous = coordinates[index - 1]
            following = coordinates[(index + 1) % len(coordinates)]
            incoming = point - previous
            outgoing = following - point
            cross = float(
                incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            )
            if abs(cross) <= 1.0e-9 and float(np.dot(incoming, outgoing)) > 0.0:
                changed = True
                continue
            simplified.append(point)
        coordinates = simplified
    coordinates = [
        point
        for index, point in enumerate(coordinates)
        if np.linalg.norm(point - coordinates[index - 1]) > 1.0e-9
    ]
    start = min(
        range(len(coordinates)),
        key=lambda index: (
            round(float(coordinates[index][0]), 9),
            round(float(coordinates[index][1]), 9),
        ),
    )
    return coordinates[start:] + coordinates[:start]

def _wing_ids_at(
    point: np.ndarray,
    wing_footprints: dict[str, Polygon],
    tolerance: float,
) -> tuple[str, ...]:
    location = Point(float(point[0]), float(point[1]))
    matches = [
        wing_id
        for wing_id, polygon in wing_footprints.items()
        if polygon.boundary.distance(location) <= tolerance
    ]
    return tuple(sorted(matches))

def _feature_name(
    point: np.ndarray,
    centroid: np.ndarray,
    reflex: bool,
    wing_ids: tuple[str, ...],
    feature_id: str,
) -> str:
    if reflex:
        if len(wing_ids) >= 2:
            return f"inside corner where wings {' and '.join(wing_ids)} meet"
        return f"inside corner {feature_id}"
    direction = compass_name(point - centroid)
    if wing_ids:
        return f"{direction} outer corner of wing {'/'.join(wing_ids)}"
    return f"{direction} outer corner"

def build_model(
    snapshot: ProcessedMapInput, config: LocalizerConfig
) -> BuildingModel:
    """Validate and reconstruct a single connected finite building model."""
    if snapshot.frame_id != config.expected_frame_id:
        raise BuildingGeometryError(
            f"Expected frame '{config.expected_frame_id}', got '{snapshot.frame_id}'"
        )
    if (
        not math.isfinite(snapshot.building_height_m)
        or snapshot.building_height_m <= 0.0
    ):
        raise BuildingGeometryError("Building height must be positive")
    if (
        not math.isfinite(snapshot.building_height_stddev_m)
        or snapshot.building_height_stddev_m < 0.0
    ):
        raise BuildingGeometryError("Building height standard deviation is invalid")
    if not snapshot.wings:
        raise BuildingGeometryError("At least one building wing is required")

    normalized_planes = tuple(
        normalize_plane(plane, config.condition_epsilon) for plane in snapshot.planes
    )
    plane_by_id: dict[str, PlaneInput] = {}
    for plane in normalized_planes:
        if plane.id in plane_by_id:
            raise BuildingGeometryError(f"Duplicate plane id '{plane.id}'")
        plane_by_id[plane.id] = plane
    try:
        ground = plane_by_id[snapshot.ground_plane_id]
    except KeyError as error:
        raise BuildingGeometryError(
            f"Unknown ground plane '{snapshot.ground_plane_id}'"
        ) from error

    ground_limit = math.cos(math.radians(config.wall_vertical_tolerance_deg))
    if float(np.dot(ground.normal, np.array([0.0, 0.0, 1.0]))) < ground_limit:
        raise BuildingGeometryError("Ground normal must point down along mission FRD +z")
    if abs(float(ground.normal[2])) <= config.condition_epsilon:
        raise BuildingGeometryError("Ground plane cannot be lifted into mission x/y")

    wing_ids = [wing.id for wing in snapshot.wings]
    if len(wing_ids) != len(set(wing_ids)):
        raise BuildingGeometryError("Building wing ids must be unique")
    wing_footprints = {
        wing.id: _wing_polygon(snapshot, plane_by_id, ground, index, config)
        for index, wing in enumerate(snapshot.wings)
    }
    footprint = unary_union(list(wing_footprints.values()))
    if isinstance(footprint, MultiPolygon):
        expanded = unary_union(
            [
                polygon.buffer(
                    config.wing_join_tolerance_m / 2.0, join_style=2
                )
                for polygon in wing_footprints.values()
            ]
        )
        if isinstance(expanded, MultiPolygon):
            raise BuildingGeometryError("Building wings do not form one connected footprint")
        footprint = expanded.buffer(
            -config.wing_join_tolerance_m / 2.0, join_style=2
        )
    if not isinstance(footprint, Polygon) or footprint.is_empty:
        raise BuildingGeometryError("Building union did not produce one polygon")
    if not footprint.is_valid:
        raise BuildingGeometryError("Merged building footprint is invalid")
    if footprint.interiors:
        raise BuildingGeometryError("Building footprint holes/courtyards are unsupported")
    footprint = orient(footprint, sign=1.0)

    model_without_features = BuildingModel(
        ground_plane=ground,
        height_m=snapshot.building_height_m,
        footprint=footprint,
        wing_footprints=wing_footprints,
        corners=(),
        walls=(),
    )
    ring = _canonical_ring(footprint)
    centroid = np.array([footprint.centroid.x, footprint.centroid.y], dtype=float)
    feature_tolerance = max(config.wing_join_tolerance_m, 1.0e-6)
    corners: list[CornerFeature] = []
    for index, point in enumerate(ring):
        previous = ring[index - 1]
        following = ring[(index + 1) % len(ring)]
        incoming = point - previous
        outgoing = following - point
        cross = float(incoming[0] * outgoing[1] - incoming[1] * outgoing[0])
        reflex = cross < -config.condition_epsilon
        feature_id = f"corner_{index}"
        point_wing_ids = _wing_ids_at(point, wing_footprints, feature_tolerance)
        corners.append(
            CornerFeature(
                id=feature_id,
                point_2d=point,
                point_3d=model_without_features.ground_point(point),
                reflex=reflex,
                wing_ids=point_wing_ids,
                name=_feature_name(
                    point, centroid, reflex, point_wing_ids, feature_id
                ),
            )
        )

    walls: list[WallFeature] = []
    for index, start in enumerate(ring):
        end = ring[(index + 1) % len(ring)]
        edge = end - start
        edge_length = float(np.linalg.norm(edge))
        if edge_length <= config.condition_epsilon:
            raise BuildingGeometryError("Building footprint contains a zero-length wall")
        outward_2d = np.array([edge[1], -edge[0]], dtype=float) / edge_length
        ground_normal = ground.normal
        edge_3d = (
            model_without_features.ground_point(end)
            - model_without_features.ground_point(start)
        )
        outward_3d = np.cross(edge_3d, ground_normal)
        outward_3d /= np.linalg.norm(outward_3d)
        midpoint = (start + end) / 2.0
        wall_wing_ids = _wing_ids_at(
            midpoint, wing_footprints, feature_tolerance
        )
        direction = compass_name(outward_2d)
        if wall_wing_ids:
            name = f"{direction}-facing wall of wing {'/'.join(wall_wing_ids)}"
        else:
            name = f"{direction}-facing exterior wall"
        walls.append(
            WallFeature(
                id=f"wall_{index}",
                start_2d=start,
                end_2d=end,
                start_3d=model_without_features.ground_point(start),
                end_3d=model_without_features.ground_point(end),
                outward_3d=outward_3d,
                wing_ids=wall_wing_ids,
                name=name,
                start_corner_id=corners[index].id,
                end_corner_id=corners[(index + 1) % len(corners)].id,
            )
        )

    return replace(
        model_without_features,
        corners=tuple(corners),
        walls=tuple(walls),
    )

def closest_point_on_wall(
    point: np.ndarray, wall: WallFeature, model: BuildingModel
) -> tuple[np.ndarray, float, float, float]:
    """Return closest wall point, distance, along coordinate, and height."""
    along_axis = wall.end_3d - wall.start_3d
    wall_length = float(np.linalg.norm(along_axis))
    along_axis /= wall_length
    up_axis = -model.ground_plane.normal
    relative = np.asarray(point, dtype=float) - wall.start_3d
    along = float(np.clip(np.dot(relative, along_axis), 0.0, wall_length))
    height = float(np.clip(np.dot(relative, up_axis), 0.0, model.height_m))
    closest = wall.start_3d + along * along_axis + height * up_axis
    return closest, float(np.linalg.norm(point - closest)), along, height

def closest_point_on_ground(
    point: np.ndarray, model: BuildingModel
) -> tuple[np.ndarray, float]:
    """Project a point onto the shared ground plane."""
    signed_distance = float(
        np.dot(model.ground_plane.normal, point) + model.ground_plane.offset
    )
    closest = np.asarray(point, dtype=float) - signed_distance * model.ground_plane.normal
    return closest, abs(signed_distance)
