"""Target-to-building localization, descriptions and uncertainty propagation."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace

import numpy as np
from shapely.geometry import Point
from utils.src.types import Plane, Vector3D

from .geometry import (
    BuildingGeometryError,
    build_model,
    closest_point_on_ground,
    closest_point_on_wall,
    compass_name,
    validate_covariance,
)
from .models import (
    AnchorType,
    BuildingModel,
    BuildingWingInput,
    LocalizationBatchResult,
    LocalizationStatus,
    LocalizedTargetResult,
    LocalizerConfig,
    PlaneInput,
    ProcessedMapInput,
    ReferenceMeasurementResult,
    Relation,
    SpatialTargetInput,
    SurfaceType,
    WallFeature,
)


_COMPASS_RELATIONS = {
    "north": Relation.NORTH_OF,
    "northeast": Relation.NORTHEAST_OF,
    "east": Relation.EAST_OF,
    "southeast": Relation.SOUTHEAST_OF,
    "south": Relation.SOUTH_OF,
    "southwest": Relation.SOUTHWEST_OF,
    "west": Relation.WEST_OF,
    "northwest": Relation.NORTHWEST_OF,
}


@dataclass(frozen=True)
class _SurfaceCandidate:
    id: str
    surface_type: SurfaceType
    distance_m: float
    snapped_position: np.ndarray
    wall: WallFeature | None = None
    along_m: float = 0.0
    height_m: float = 0.0


def _source_position(target: SpatialTargetInput) -> np.ndarray:
    position = np.asarray(target.position, dtype=float)
    if position.shape == (3,) and np.all(np.isfinite(position)):
        return position.copy()
    return np.zeros(3, dtype=float)


def _failure(
    target: SpatialTargetInput,
    status: LocalizationStatus,
    reason: str,
) -> LocalizedTargetResult:
    return LocalizedTargetResult(
        target_id=target.id,
        colour=target.colour,
        status=status,
        reason=reason,
        source_position=_source_position(target),
    )


def _relation_for_vector(vector_2d: np.ndarray) -> tuple[Relation, str]:
    direction = compass_name(vector_2d)
    if direction == "at":
        return Relation.DISTANCE, "from"
    return _COMPASS_RELATIONS[direction], f"{direction} of"


def _project_to_segment(
    point: np.ndarray, start: np.ndarray, end: np.ndarray
) -> tuple[np.ndarray, float, float]:
    edge = end - start
    squared_length = float(np.dot(edge, edge))
    if squared_length <= 1.0e-12:
        return start.copy(), float(np.linalg.norm(point - start)), 0.0
    fraction = float(np.clip(np.dot(point - start, edge) / squared_length, 0.0, 1.0))
    projected = start + fraction * edge
    return projected, float(np.linalg.norm(point - projected)), fraction


def _format_colour(colour: str) -> str:
    return colour.strip().replace("_", " ").lower()


def _validate_target(target: SpatialTargetInput) -> str | None:
    if not target.id:
        return "Target id must be non-empty"
    if not target.colour:
        return "Target colour must be non-empty"
    position = np.asarray(target.position, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        return "Target position must contain three finite values"
    try:
        validate_covariance(target.covariance, 3, f"Target '{target.id}'")
    except ValueError as error:
        return str(error)
    return None


def _surface_candidates(
    target: SpatialTargetInput, model: BuildingModel
) -> list[_SurfaceCandidate]:
    ground_point, ground_distance = closest_point_on_ground(target.position, model)
    candidates = [
        _SurfaceCandidate(
            id="ground",
            surface_type=SurfaceType.GROUND,
            distance_m=ground_distance,
            snapped_position=ground_point,
        )
    ]
    for wall in model.walls:
        closest, distance, along, height = closest_point_on_wall(
            target.position, wall, model
        )
        candidates.append(
            _SurfaceCandidate(
                id=wall.id,
                surface_type=SurfaceType.WALL,
                distance_m=distance,
                snapped_position=closest,
                wall=wall,
                along_m=along,
                height_m=height,
            )
        )
    return sorted(candidates, key=lambda candidate: (candidate.distance_m, candidate.id))


def _wall_result(
    target: SpatialTargetInput,
    candidate: _SurfaceCandidate,
    model: BuildingModel,
    config: LocalizerConfig,
) -> LocalizedTargetResult:
    wall = candidate.wall
    assert wall is not None
    corner_by_id = {corner.id: corner for corner in model.corners}
    start_corner = corner_by_id[wall.start_corner_id]
    end_corner = corner_by_id[wall.end_corner_id]
    wall_length = float(np.linalg.norm(wall.end_3d - wall.start_3d))
    start_distance = candidate.along_m
    end_distance = wall_length - candidate.along_m
    if abs(start_distance - end_distance) <= config.anchor_tie_tolerance_m:
        return _failure(
            target,
            LocalizationStatus.AMBIGUOUS_ANCHOR,
            f"Target is equally close to both ends of wall '{wall.id}'",
        )
    if start_distance < end_distance:
        corner = start_corner
        lateral_distance = start_distance
    else:
        corner = end_corner
        lateral_distance = end_distance

    vertical_from_ground = candidate.height_m
    vertical_from_roof = model.height_m - candidate.height_m
    if vertical_from_ground <= vertical_from_roof:
        vertical_relation = Relation.ABOVE
        vertical_reference = "ground"
        vertical_distance = vertical_from_ground
        vertical_phrase = f"{vertical_distance:.1f} m above the ground"
    else:
        vertical_relation = Relation.BELOW
        vertical_reference = "roof"
        vertical_distance = vertical_from_roof
        vertical_phrase = f"{vertical_distance:.1f} m below the roofline"

    relation, relation_phrase = _relation_for_vector(
        candidate.snapped_position[:2] - corner.point_2d
    )
    if relation is Relation.DISTANCE:
        lateral_phrase = f"{lateral_distance:.1f} m from {corner.name}"
    else:
        lateral_phrase = (
            f"{lateral_distance:.1f} m {relation_phrase} {corner.name}"
        )
    colour = _format_colour(target.colour)
    body = (
        f"Target {target.id} is on the {wall.name}, {vertical_phrase} and "
        f"{lateral_phrase}. The colour is {colour}."
    )
    return LocalizedTargetResult(
        target_id=target.id,
        colour=target.colour,
        status=LocalizationStatus.OK,
        reason="",
        source_position=np.asarray(target.position, dtype=float).copy(),
        snapped_position=candidate.snapped_position.copy(),
        surface_id=wall.id,
        surface_type=SurfaceType.WALL,
        surface_name=wall.name,
        anchor_id=corner.id,
        anchor_type=(
            AnchorType.INNER_CORNER if corner.reflex else AnchorType.OUTER_CORNER
        ),
        reference_names=(vertical_reference, corner.name),
        measurements=[
            ReferenceMeasurementResult(
                reference_id=vertical_reference,
                relation=vertical_relation,
                distance_m=vertical_distance,
            ),
            ReferenceMeasurementResult(
                reference_id=corner.id,
                relation=relation,
                distance_m=lateral_distance,
            ),
        ],
        description_body=body,
    )


def _near_ground_result(
    target: SpatialTargetInput,
    snapped_position: np.ndarray,
    wall: WallFeature,
    wall_distance: float,
    wall_projection: np.ndarray,
    wall_fraction: float,
    model: BuildingModel,
    config: LocalizerConfig,
) -> LocalizedTargetResult:
    corner_by_id = {corner.id: corner for corner in model.corners}
    start_corner = corner_by_id[wall.start_corner_id]
    end_corner = corner_by_id[wall.end_corner_id]
    wall_length = float(np.linalg.norm(wall.end_2d - wall.start_2d))
    start_distance = wall_fraction * wall_length
    end_distance = (1.0 - wall_fraction) * wall_length
    if abs(start_distance - end_distance) <= config.anchor_tie_tolerance_m:
        return _failure(
            target,
            LocalizationStatus.AMBIGUOUS_ANCHOR,
            f"Ground target is equally close to both ends of wall '{wall.id}'",
        )
    if start_distance < end_distance:
        corner = start_corner
        along_distance = start_distance
    else:
        corner = end_corner
        along_distance = end_distance
    relation, relation_phrase = _relation_for_vector(
        wall_projection - corner.point_2d
    )
    if relation is Relation.DISTANCE:
        along_phrase = f"{along_distance:.1f} m from {corner.name}"
    else:
        along_phrase = f"{along_distance:.1f} m {relation_phrase} {corner.name}"
    colour = _format_colour(target.colour)
    body = (
        f"Target {target.id} is on the ground, {wall_distance:.1f} m out from "
        f"the {wall.name} and {along_phrase}. The colour is {colour}."
    )
    return LocalizedTargetResult(
        target_id=target.id,
        colour=target.colour,
        status=LocalizationStatus.OK,
        reason="",
        source_position=np.asarray(target.position, dtype=float).copy(),
        snapped_position=snapped_position.copy(),
        surface_id="ground",
        surface_type=SurfaceType.GROUND,
        surface_name="ground",
        anchor_id=corner.id,
        anchor_type=(
            AnchorType.INNER_CORNER if corner.reflex else AnchorType.OUTER_CORNER
        ),
        reference_names=(wall.name, corner.name),
        measurements=[
            ReferenceMeasurementResult(
                reference_id=wall.id,
                relation=Relation.OUT_FROM,
                distance_m=wall_distance,
            ),
            ReferenceMeasurementResult(
                reference_id=corner.id,
                relation=relation,
                distance_m=along_distance,
            ),
        ],
        description_body=body,
    )


def _far_ground_result(
    target: SpatialTargetInput,
    snapped_position: np.ndarray,
    model: BuildingModel,
    config: LocalizerConfig,
) -> LocalizedTargetResult:
    if len(model.corners) < 2:
        return _failure(
            target,
            LocalizationStatus.AMBIGUOUS_ANCHOR,
            "At least two building corners are required for a far-ground target",
        )
    distances = sorted(
        (
            float(np.linalg.norm(snapped_position - corner.point_3d)),
            corner.id,
            corner,
        )
        for corner in model.corners
    )
    if (
        len(distances) >= 3
        and abs(distances[1][0] - distances[2][0])
        <= config.anchor_tie_tolerance_m
    ):
        return _failure(
            target,
            LocalizationStatus.AMBIGUOUS_ANCHOR,
            "Far-ground target has an ambiguous second corner",
        )
    first_distance, _, first_corner = distances[0]
    second_distance, _, second_corner = distances[1]
    centroid = np.array(
        [model.footprint.centroid.x, model.footprint.centroid.y], dtype=float
    )
    side = compass_name(snapped_position[:2] - centroid)
    colour = _format_colour(target.colour)
    body = (
        f"Target {target.id} is on the ground {side} of the building, "
        f"{first_distance:.1f} m from {first_corner.name} and "
        f"{second_distance:.1f} m from {second_corner.name}. "
        f"The colour is {colour}."
    )
    return LocalizedTargetResult(
        target_id=target.id,
        colour=target.colour,
        status=LocalizationStatus.OK,
        reason="",
        source_position=np.asarray(target.position, dtype=float).copy(),
        snapped_position=snapped_position.copy(),
        surface_id="ground",
        surface_type=SurfaceType.GROUND,
        surface_name="ground",
        anchor_id=f"{first_corner.id},{second_corner.id}",
        anchor_type=AnchorType.TWO_CORNERS,
        reference_names=(first_corner.name, second_corner.name),
        measurements=[
            ReferenceMeasurementResult(
                reference_id=first_corner.id,
                relation=Relation.DISTANCE,
                distance_m=first_distance,
            ),
            ReferenceMeasurementResult(
                reference_id=second_corner.id,
                relation=Relation.DISTANCE,
                distance_m=second_distance,
            ),
        ],
        description_body=body,
    )


def _ground_result(
    target: SpatialTargetInput,
    candidate: _SurfaceCandidate,
    model: BuildingModel,
    config: LocalizerConfig,
) -> LocalizedTargetResult:
    snapped_2d = candidate.snapped_position[:2]
    if model.footprint.contains(Point(float(snapped_2d[0]), float(snapped_2d[1]))):
        return _failure(
            target,
            LocalizationStatus.NO_SURFACE,
            "Ground target lies inside the building footprint",
        )
    wall_candidates = []
    for wall in model.walls:
        projection, distance, fraction = _project_to_segment(
            snapped_2d, wall.start_2d, wall.end_2d
        )
        wall_candidates.append((distance, wall.id, wall, projection, fraction))
    wall_candidates.sort(key=lambda item: (item[0], item[1]))
    wall_distance, _, wall, projection, fraction = wall_candidates[0]
    if wall_distance <= config.near_wall_distance_m:
        if (
            len(wall_candidates) > 1
            and abs(wall_candidates[1][0] - wall_distance)
            <= config.anchor_tie_tolerance_m
        ):
            return _failure(
                target,
                LocalizationStatus.AMBIGUOUS_ANCHOR,
                "Ground target is equally close to multiple walls",
            )
        return _near_ground_result(
            target,
            candidate.snapped_position,
            wall,
            wall_distance,
            projection,
            fraction,
            model,
            config,
        )
    return _far_ground_result(target, candidate.snapped_position, model, config)


def _localize_target(
    target: SpatialTargetInput,
    model: BuildingModel,
    config: LocalizerConfig,
) -> LocalizedTargetResult:
    validation_error = _validate_target(target)
    if validation_error:
        return _failure(target, LocalizationStatus.INVALID_TARGET, validation_error)
    candidates = _surface_candidates(target, model)
    best = candidates[0]
    if best.distance_m > config.max_snap_distance_m:
        return _failure(
            target,
            LocalizationStatus.NO_SURFACE,
            (
                f"Closest finite surface is {best.distance_m:.3f} m away, exceeding "
                f"the {config.max_snap_distance_m:.3f} m limit"
            ),
        )
    if (
        len(candidates) > 1
        and candidates[1].distance_m - best.distance_m
        <= config.surface_tie_tolerance_m
    ):
        return _failure(
            target,
            LocalizationStatus.AMBIGUOUS_SURFACE,
            (
                f"Surfaces '{best.id}' and '{candidates[1].id}' are within "
                f"{config.surface_tie_tolerance_m:.3f} m"
            ),
        )
    if best.surface_type is SurfaceType.WALL:
        return _wall_result(target, best, model, config)
    return _ground_result(target, best, model, config)


def _result_signature(result: LocalizedTargetResult) -> tuple:
    return (
        result.status,
        result.surface_type,
        result.surface_name,
        result.anchor_type,
        result.reference_names,
        tuple(
            measurement.relation for measurement in result.measurements
        ),
    )


def _sample_plane(plane: PlaneInput, generator: np.random.Generator) -> PlaneInput:
    covariance = validate_covariance(plane.covariance, 4, f"Plane '{plane.id}'")
    mean = np.concatenate((np.asarray(plane.normal, dtype=float), [plane.offset]))
    if np.any(covariance):
        sample = generator.multivariate_normal(mean, covariance, check_valid="raise")
    else:
        sample = mean.copy()
    normal_norm = float(np.linalg.norm(sample[:3]))
    if normal_norm <= 1.0e-9:
        raise BuildingGeometryError(f"Sampled plane '{plane.id}' has a zero normal")
    normal = sample[:3] / normal_norm
    offset = float(sample[3]) / normal_norm
    if float(np.dot(normal, plane.normal)) < 0.0:
        normal = -normal
        offset = -offset
    return replace(
        plane,
        plane=Plane(
            normal=Vector3D(
                x=float(normal[0]),
                y=float(normal[1]),
                z=float(normal[2]),
            ),
            offset=offset,
        ),
        covariance=covariance,
    )


def _sample_wing(
    wing: BuildingWingInput, generator: np.random.Generator
) -> BuildingWingInput:
    boundaries = tuple(
        replace(
            boundary,
            opposite_distance_m=float(
                generator.normal(
                    boundary.opposite_distance_m, boundary.distance_stddev_m
                )
            ),
        )
        for boundary in wing.boundaries
    )
    return replace(wing, boundaries=boundaries)


def _sample_building(
    snapshot: ProcessedMapInput, generator: np.random.Generator
) -> ProcessedMapInput:
    return replace(
        snapshot,
        planes=tuple(_sample_plane(plane, generator) for plane in snapshot.planes),
        building_height_m=float(
            generator.normal(
                snapshot.building_height_m, snapshot.building_height_stddev_m
            )
        ),
        wings=tuple(_sample_wing(wing, generator) for wing in snapshot.wings),
    )


def _sample_target(
    target: SpatialTargetInput, generator: np.random.Generator
) -> SpatialTargetInput:
    covariance = validate_covariance(target.covariance, 3, f"Target '{target.id}'")
    if np.any(covariance):
        position = generator.multivariate_normal(
            np.asarray(target.position, dtype=float), covariance, check_valid="raise"
        )
    else:
        position = np.asarray(target.position, dtype=float).copy()
    return replace(target, position=position, covariance=covariance)


def format_descriptions(result: LocalizationBatchResult) -> str:
    """Return newline-separated authoritative target descriptions."""
    return "\n".join(
        target.description for target in result.targets if target.description
    )


class BuildingTargetLocalizer:
    """Localize all targets in a processed-map snapshot."""

    def __init__(self, config: LocalizerConfig | None = None) -> None:
        """Initialize the localizer with explicit or default thresholds."""
        self.config = config or LocalizerConfig()

    def localize(self, snapshot: ProcessedMapInput) -> LocalizationBatchResult:
        """Reconstruct the map, localize targets, and propagate uncertainty."""
        try:
            model = build_model(snapshot, self.config)
        except (BuildingGeometryError, ValueError, np.linalg.LinAlgError) as error:
            return LocalizationBatchResult(
                map_valid=False,
                map_error=str(error),
                targets=[
                    _failure(target, LocalizationStatus.MAP_INVALID, str(error))
                    for target in snapshot.targets
                ],
            )

        duplicate_ids = {
            target_id
            for target_id, count in Counter(
                target.id for target in snapshot.targets if target.id
            ).items()
            if count > 1
        }
        results = []
        for target in snapshot.targets:
            if target.id in duplicate_ids:
                results.append(
                    _failure(
                        target,
                        LocalizationStatus.INVALID_TARGET,
                        f"Duplicate target id '{target.id}'",
                    )
                )
            else:
                results.append(_localize_target(target, model, self.config))

        self._propagate_uncertainty(snapshot, results)
        for result in results:
            if result.status is LocalizationStatus.OK:
                result.description = (
                    f"{result.description_body} (±{result.uncertainty_95_m:.1f} m)"
                )
        return LocalizationBatchResult(map_valid=True, map_error="", targets=results)

    def _propagate_uncertainty(
        self,
        snapshot: ProcessedMapInput,
        nominal_results: list[LocalizedTargetResult],
    ) -> None:
        sample_count = self.config.uncertainty_samples
        if sample_count <= 0:
            return
        valid_indices = [
            index
            for index, result in enumerate(nominal_results)
            if result.status is LocalizationStatus.OK
        ]
        building_has_uncertainty = (
            snapshot.building_height_stddev_m > 0.0
            or any(np.any(plane.covariance) for plane in snapshot.planes)
            or any(
                boundary.distance_stddev_m > 0.0
                for wing in snapshot.wings
                for boundary in wing.boundaries
            )
        )
        valid_indices = [
            index
            for index in valid_indices
            if building_has_uncertainty
            or np.any(snapshot.targets[index].covariance)
        ]
        if not valid_indices:
            return

        generator = np.random.default_rng(self.config.uncertainty_seed)
        unstable = {index: 0 for index in valid_indices}
        measurement_deviations = {
            index: [[] for _ in nominal_results[index].measurements]
            for index in valid_indices
        }
        snapped_deviations = {index: [] for index in valid_indices}

        for _ in range(sample_count):
            try:
                sampled_snapshot = _sample_building(snapshot, generator)
                sampled_model = build_model(sampled_snapshot, self.config)
            except (BuildingGeometryError, ValueError, np.linalg.LinAlgError):
                for index in valid_indices:
                    unstable[index] += 1
                continue

            for index in valid_indices:
                nominal = nominal_results[index]
                try:
                    sampled_target = _sample_target(snapshot.targets[index], generator)
                    sampled = _localize_target(
                        sampled_target, sampled_model, self.config
                    )
                except (ValueError, np.linalg.LinAlgError):
                    unstable[index] += 1
                    continue
                if _result_signature(sampled) != _result_signature(nominal):
                    unstable[index] += 1
                    continue
                snapped_deviations[index].append(
                    float(
                        np.linalg.norm(
                            sampled.snapped_position - nominal.snapped_position
                        )
                    )
                )
                for measurement_index, measurement in enumerate(
                    sampled.measurements
                ):
                    measurement_deviations[index][measurement_index].append(
                        abs(
                            measurement.distance_m
                            - nominal.measurements[measurement_index].distance_m
                        )
                    )

        for index in valid_indices:
            result = nominal_results[index]
            unstable_fraction = unstable[index] / sample_count
            if unstable_fraction > self.config.max_unstable_sample_fraction:
                result.status = LocalizationStatus.UNSTABLE_GEOMETRY
                result.reason = (
                    f"Surface or anchor changed in {unstable_fraction:.1%} of "
                    "uncertainty samples"
                )
                result.description = ""
                result.description_body = ""
                result.measurements = []
                result.surface_id = ""
                result.surface_type = SurfaceType.NONE
                result.surface_name = ""
                result.anchor_id = ""
                result.anchor_type = AnchorType.NONE
                result.reference_names = ()
                continue

            uncertainty_values = []
            for measurement, deviations in zip(
                result.measurements, measurement_deviations[index]
            ):
                if deviations:
                    measurement.uncertainty_95_m = float(
                        np.percentile(deviations, 95.0)
                    )
                    uncertainty_values.append(measurement.uncertainty_95_m)
            if snapped_deviations[index]:
                uncertainty_values.append(
                    float(np.percentile(snapped_deviations[index], 95.0))
                )
            result.uncertainty_95_m = max(uncertainty_values, default=0.0)
