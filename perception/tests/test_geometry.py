"""Building reconstruction tests."""

from dataclasses import replace

import numpy as np
import pytest
from utils.src.types import Plane as SharedPlane

from building_target_localizer.geometry import (
    BuildingGeometryError,
    build_model,
    compass_name,
)
from building_target_localizer.models import (
    BuildingWingInput,
    LocalizerConfig,
    ProcessedMapInput,
    WingBoundaryInput,
)

from .helpers import l_snapshot, plane, rectangular_snapshot


def config() -> LocalizerConfig:
    return LocalizerConfig(uncertainty_samples=0)


def test_compass_name_uses_mission_frd_axes():
    assert compass_name(np.array([1.0, 0.0])) == "north"
    assert compass_name(np.array([0.0, 1.0])) == "east"
    assert compass_name(np.array([-1.0, -1.0])) == "southwest"


def test_rectangular_wing_reconstructs_unseen_faces_and_roof():
    snapshot = rectangular_snapshot()
    assert isinstance(snapshot.planes[0].plane, SharedPlane)
    model = build_model(snapshot, config())
    assert model.footprint.area == pytest.approx(60.0)
    assert len(model.walls) == 4
    assert len(model.corners) == 4
    assert not any(corner.reflex for corner in model.corners)
    assert model.ground_point(np.array([2.0, 3.0])).tolist() == [2.0, 3.0, 0.0]
    assert model.roof_point(np.array([2.0, 3.0])).tolist() == [2.0, 3.0, -6.0]
    assert {wall.name.split("-")[0] for wall in model.walls} == {
        "north",
        "east",
        "south",
        "west",
    }


def test_l_wings_merge_internal_walls_and_keep_reflex_corner():
    model = build_model(l_snapshot(), config())
    assert model.footprint.area == pytest.approx(64.0)
    assert len(model.walls) == 6
    reflex = [corner for corner in model.corners if corner.reflex]
    assert len(reflex) == 1
    assert reflex[0].point_2d.tolist() == pytest.approx([4.0, 4.0])
    assert "inside corner" in reflex[0].name


def test_rotated_l_wings_keep_union_topology():
    angle = np.deg2rad(27.0)
    first_normal = np.array([-np.cos(angle), -np.sin(angle), 0.0])
    second_normal = np.array([np.sin(angle), -np.cos(angle), 0.0])
    planes = (
        plane("ground", (0.0, 0.0, 1.0), 0.0),
        plane("a_first", tuple(first_normal), 0.0),
        plane("a_second", tuple(second_normal), 0.0),
        plane("b_first", tuple(first_normal), 0.0),
        plane("b_second", tuple(second_normal), 0.0),
    )
    snapshot = ProcessedMapInput(
        frame_id="mission_frd",
        planes=planes,
        ground_plane_id="ground",
        building_height_m=6.0,
        building_height_stddev_m=0.0,
        wings=(
            BuildingWingInput(
                id="horizontal",
                boundaries=(
                    WingBoundaryInput("a_first", 10.0, 0.0),
                    WingBoundaryInput("a_second", 4.0, 0.0),
                ),
            ),
            BuildingWingInput(
                id="vertical",
                boundaries=(
                    WingBoundaryInput("b_first", 4.0, 0.0),
                    WingBoundaryInput("b_second", 10.0, 0.0),
                ),
            ),
        ),
        targets=(),
    )
    model = build_model(snapshot, config())
    assert model.footprint.area == pytest.approx(64.0)
    assert len(model.walls) == 6
    assert sum(corner.reflex for corner in model.corners) == 1


def test_rotated_rectangle_preserves_area():
    angle = np.deg2rad(30.0)
    first_normal = np.array([-np.cos(angle), -np.sin(angle), 0.0])
    second_normal = np.array([np.sin(angle), -np.cos(angle), 0.0])
    planes = (
        plane("ground", (0.0, 0.0, 1.0), 0.0),
        plane("first", tuple(first_normal), 0.0),
        plane("second", tuple(second_normal), 0.0),
    )
    snapshot = ProcessedMapInput(
        frame_id="mission_frd",
        planes=planes,
        ground_plane_id="ground",
        building_height_m=4.0,
        building_height_stddev_m=0.0,
        wings=(
            BuildingWingInput(
                id="rotated",
                boundaries=(
                    WingBoundaryInput("first", 8.0, 0.0),
                    WingBoundaryInput("second", 3.0, 0.0),
                ),
            ),
        ),
        targets=(),
    )
    model = build_model(snapshot, config())
    assert model.footprint.area == pytest.approx(24.0)


def test_slightly_tilted_ground_keeps_walls_and_roof_consistent():
    angle = np.deg2rad(2.0)
    ground_normal = np.array([np.sin(angle), 0.0, np.cos(angle)])
    south_normal = np.array([-np.cos(angle), 0.0, np.sin(angle)])
    snapshot = ProcessedMapInput(
        frame_id="mission_frd",
        planes=(
            plane("ground", tuple(ground_normal), 0.0),
            plane("south", tuple(south_normal), 0.0),
            plane("west", (0.0, -1.0, 0.0), 0.0),
        ),
        ground_plane_id="ground",
        building_height_m=6.0,
        building_height_stddev_m=0.0,
        wings=(
            BuildingWingInput(
                id="tilted",
                boundaries=(
                    WingBoundaryInput("south", 10.0, 0.0),
                    WingBoundaryInput("west", 6.0, 0.0),
                ),
            ),
        ),
        targets=(),
    )
    model = build_model(snapshot, config())
    for corner in model.corners:
        assert np.dot(ground_normal, corner.point_3d) == pytest.approx(0.0)
        roof = model.roof_point(corner.point_2d)
        assert np.linalg.norm(roof - corner.point_3d) == pytest.approx(6.0)
    for wall in model.walls:
        edge = wall.end_3d - wall.start_3d
        assert np.dot(edge, wall.outward_3d) == pytest.approx(0.0, abs=1.0e-9)
        assert np.dot(ground_normal, wall.outward_3d) == pytest.approx(
            0.0, abs=1.0e-9
        )


def test_t_wings_produce_two_reflex_corners():
    base = rectangular_snapshot(length=10.0, width=4.0)
    extra_planes = (
        plane("stem_south", (-1.0, 0.0, 0.0), 4.0),
        plane("stem_west", (0.0, -1.0, 0.0), 0.0),
    )
    stem = BuildingWingInput(
        id="stem",
        boundaries=(
            WingBoundaryInput("stem_south", 2.0, 0.0),
            WingBoundaryInput("stem_west", 10.0, 0.0),
        ),
    )
    snapshot = replace(
        base,
        planes=base.planes + extra_planes,
        wings=base.wings + (stem,),
    )
    model = build_model(snapshot, config())
    assert model.footprint.area == pytest.approx(52.0)
    assert sum(corner.reflex for corner in model.corners) == 2


def test_wrong_frame_is_rejected():
    snapshot = replace(rectangular_snapshot(), frame_id="map")
    with pytest.raises(BuildingGeometryError, match="Expected frame"):
        build_model(snapshot, config())


def test_disconnected_wings_are_rejected():
    base = rectangular_snapshot()
    extra_planes = (
        plane("remote_south", (-1.0, 0.0, 0.0), 30.0),
        plane("remote_west", (0.0, -1.0, 0.0), 30.0),
    )
    remote = BuildingWingInput(
        id="remote",
        boundaries=(
            WingBoundaryInput("remote_south", 2.0, 0.0),
            WingBoundaryInput("remote_west", 2.0, 0.0),
        ),
    )
    snapshot = replace(
        base,
        planes=base.planes + extra_planes,
        wings=base.wings + (remote,),
    )
    with pytest.raises(BuildingGeometryError, match="connected"):
        build_model(snapshot, config())


def test_nonvertical_and_nonorthogonal_walls_are_rejected():
    base = rectangular_snapshot()
    tilted = plane(
        base.planes[1].id,
        (-0.9, 0.0, 0.4),
        base.planes[1].offset,
        base.planes[1].covariance,
    )
    with pytest.raises(BuildingGeometryError, match="not vertical"):
        build_model(replace(base, planes=(base.planes[0], tilted, base.planes[2])), config())

    diagonal = plane(
        base.planes[2].id,
        (-0.5, -0.5, 0.0),
        base.planes[2].offset,
        base.planes[2].covariance,
    )
    with pytest.raises(BuildingGeometryError, match="near-orthogonal"):
        build_model(
            replace(base, planes=(base.planes[0], base.planes[1], diagonal)),
            config(),
        )
