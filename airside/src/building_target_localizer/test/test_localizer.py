"""Target localization, text generation, and uncertainty tests."""

from dataclasses import replace

import numpy as np
import pytest

from building_target_localizer.localizer import BuildingTargetLocalizer
from building_target_localizer.models import (
    LocalizationStatus,
    LocalizerConfig,
    Relation,
    SurfaceType,
)

from .helpers import l_snapshot, rectangular_snapshot, target


def localizer(**overrides) -> BuildingTargetLocalizer:
    return BuildingTargetLocalizer(
        LocalizerConfig(uncertainty_samples=0, **overrides)
    )


def test_wall_target_snaps_and_uses_ground_plus_corner():
    snapshot = rectangular_snapshot(
        targets=(target("A", (10.2, 2.0, -2.0), colour="BLUE"),)
    )
    result = localizer().localize(snapshot)
    localized = result.targets[0]
    assert result.map_valid
    assert localized.status is LocalizationStatus.OK
    assert localized.surface_type is SurfaceType.WALL
    assert localized.snapped_position.tolist() == pytest.approx([10.0, 2.0, -2.0])
    assert [item.relation for item in localized.measurements] == [
        Relation.ABOVE,
        Relation.EAST_OF,
    ]
    assert "2.0 m above the ground" in localized.description
    assert "2.0 m east of" in localized.description
    assert localized.description.endswith("The colour is blue. (±0.0 m)")


def test_wall_target_uses_roof_when_it_is_closer():
    snapshot = rectangular_snapshot(targets=(target("A", (10.0, 2.0, -5.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.OK
    assert localized.measurements[0].relation is Relation.BELOW
    assert localized.measurements[0].distance_m == pytest.approx(1.0)
    assert "1.0 m below the roofline" in localized.description


def test_wall_target_can_anchor_to_an_inside_corner():
    snapshot = l_snapshot(targets=(target("D", (4.0, 5.0, -2.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.OK
    assert localized.anchor_type.name == "INNER_CORNER"
    assert "inside corner" in localized.description


def test_ground_target_near_wall_uses_wall_and_corner():
    snapshot = rectangular_snapshot(targets=(target("B", (4.0, -2.0, 0.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.OK
    assert localized.surface_type is SurfaceType.GROUND
    assert localized.measurements[0].relation is Relation.OUT_FROM
    assert localized.measurements[0].distance_m == pytest.approx(2.0)
    assert "2.0 m out from the west-facing wall" in localized.description
    assert "4.0 m north of" in localized.description


def test_ground_target_far_from_wall_uses_two_corners_and_side():
    snapshot = rectangular_snapshot(targets=(target("C", (5.0, 15.0, 0.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.OK
    assert localized.anchor_id.count(",") == 1
    assert all(
        measurement.relation is Relation.DISTANCE
        for measurement in localized.measurements
    )
    assert "ground east of the building" in localized.description


def test_noisy_target_over_snap_limit_is_rejected():
    snapshot = rectangular_snapshot(targets=(target("bad", (5.0, 3.0, -3.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.NO_SURFACE
    assert localized.description == ""


def test_infinite_plane_extension_is_not_treated_as_a_finite_wall():
    snapshot = rectangular_snapshot(targets=(target("outside", (11.0, 7.0, -3.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.NO_SURFACE


def test_surface_and_anchor_ties_are_reported():
    surface_tie = rectangular_snapshot(
        targets=(target("surface", (10.02, 1.0, -0.02)),)
    )
    assert (
        localizer().localize(surface_tie).targets[0].status
        is LocalizationStatus.AMBIGUOUS_SURFACE
    )

    anchor_tie = rectangular_snapshot(
        targets=(target("anchor", (10.0, 3.0, -3.0)),)
    )
    assert (
        localizer().localize(anchor_tie).targets[0].status
        is LocalizationStatus.AMBIGUOUS_ANCHOR
    )


def test_ground_target_inside_building_is_rejected():
    snapshot = rectangular_snapshot(targets=(target("inside", (5.0, 3.0, 0.0)),))
    localized = localizer().localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.NO_SURFACE
    assert "inside" in localized.reason


def test_partial_results_keep_valid_targets():
    snapshot = rectangular_snapshot(
        targets=(
            target("valid", (10.0, 2.0, -2.0)),
            target("invalid", (np.nan, 0.0, 0.0)),
        )
    )
    result = localizer().localize(snapshot)
    assert result.map_valid
    assert result.targets[0].status is LocalizationStatus.OK
    assert result.targets[1].status is LocalizationStatus.INVALID_TARGET


def test_invalid_map_marks_every_target_map_invalid():
    snapshot = replace(
        rectangular_snapshot(targets=(target("A", (10.0, 2.0, -2.0)),)),
        frame_id="map",
    )
    result = localizer().localize(snapshot)
    assert not result.map_valid
    assert result.targets[0].status is LocalizationStatus.MAP_INVALID


def test_uncertainty_is_deterministic_and_increases_from_covariance():
    covariance = np.diag([0.01**2, 0.02**2, 0.03**2])
    snapshot = rectangular_snapshot(
        targets=(target("A", (10.0, 2.0, -2.0), covariance=covariance),)
    )
    configured = BuildingTargetLocalizer(
        LocalizerConfig(uncertainty_samples=100, uncertainty_seed=97)
    )
    first = configured.localize(snapshot).targets[0]
    second = configured.localize(snapshot).targets[0]
    assert first.status is LocalizationStatus.OK
    assert first.uncertainty_95_m > 0.0
    assert first.uncertainty_95_m == second.uncertainty_95_m
    assert first.description == second.description


def test_plane_covariance_propagates_to_output_uncertainty():
    snapshot = rectangular_snapshot(
        targets=(target("A", (10.0, 2.0, -2.0)),)
    )
    plane_covariance = np.zeros((4, 4))
    plane_covariance[1, 1] = 0.005**2
    plane_covariance[3, 3] = 0.01**2
    south_wall = replace(snapshot.planes[1], covariance=plane_covariance)
    snapshot = replace(
        snapshot,
        planes=(snapshot.planes[0], south_wall, snapshot.planes[2]),
    )
    configured = BuildingTargetLocalizer(
        LocalizerConfig(uncertainty_samples=100, uncertainty_seed=97)
    )
    localized = configured.localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.OK
    assert localized.uncertainty_95_m > 0.0


def test_unstable_samples_remove_authoritative_description():
    covariance = np.diag([1.0, 0.0, 0.0])
    snapshot = rectangular_snapshot(
        targets=(target("A", (10.0, 2.0, -2.0), covariance=covariance),)
    )
    configured = BuildingTargetLocalizer(
        LocalizerConfig(uncertainty_samples=100, uncertainty_seed=97)
    )
    localized = configured.localize(snapshot).targets[0]
    assert localized.status is LocalizationStatus.UNSTABLE_GEOMETRY
    assert localized.description == ""
    assert localized.measurements == []
