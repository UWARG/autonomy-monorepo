"""Tests for single-frame target localization."""

import math

import numpy as np
import pytest

from target_location import (
    NADIR_DOWN_MOUNT,
    CameraIntrinsics,
    CameraMount,
    locate_target_frd,
    plane_normal_frd,
    project_target_frd,
)

INTRINSICS = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
CENTRE_PIXEL = (INTRINSICS.cx, INTRINSICS.cy)
HEIGHT_M = 10.0


def rotation_zyx(yaw_rad: float, pitch_rad: float, roll_rad: float) -> np.ndarray:
    """Body->NED rotation for ZYX Tait-Bryan angles."""
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)

    r_z = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
    r_y = np.array([[cos_p, 0.0, sin_p], [0.0, 1.0, 0.0], [-sin_p, 0.0, cos_p]])
    r_x = np.array([[1.0, 0.0, 0.0], [0.0, cos_r, -sin_r], [0.0, sin_r, cos_r]])

    return r_z @ r_y @ r_x


class TestPlaneNormal:
    def test_level_drone_normal_is_body_down(self):
        assert plane_normal_frd(0.0, 0.0) == pytest.approx([0.0, 0.0, 1.0])

    @pytest.mark.parametrize("roll_rad", [-1.0, -0.3, 0.0, 0.3, 1.0])
    @pytest.mark.parametrize("pitch_rad", [-1.0, -0.3, 0.0, 0.3, 1.0])
    def test_normal_is_always_a_unit_vector(self, roll_rad, pitch_rad):
        normal = plane_normal_frd(roll_rad, pitch_rad)
        assert np.linalg.norm(normal) == pytest.approx(1.0)

    @pytest.mark.parametrize("roll_rad", [-0.4, 0.0, 0.7])
    @pytest.mark.parametrize("pitch_rad", [-0.4, 0.0, 0.7])
    @pytest.mark.parametrize("yaw_rad", [-2.0, 0.0, 1.1])
    def test_matches_third_row_of_body_to_ned_rotation(
        self, roll_rad, pitch_rad, yaw_rad
    ):
        # World-down expressed in the body frame, for any yaw.
        expected = rotation_zyx(yaw_rad, pitch_rad, roll_rad).T @ np.array(
            [0.0, 0.0, 1.0]
        )
        assert plane_normal_frd(roll_rad, pitch_rad) == pytest.approx(expected)

    def test_nose_up_tilts_gravity_aft(self):
        # Pitched nose up, "down" is behind the drone: negative forward component.
        assert plane_normal_frd(0.0, 0.5)[0] < 0.0

    def test_rejects_non_finite_attitude(self):
        with pytest.raises(ValueError):
            plane_normal_frd(float("nan"), 0.0)


class TestNadirGeometry:
    def test_centre_pixel_level_drone_is_directly_below(self):
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0)
        assert position == pytest.approx([0.0, 0.0, HEIGHT_M])

    def test_forty_five_degree_pixel_lands_one_height_to_the_right(self):
        # u offset by exactly fx is a 45 degree ray in the image's x direction, which
        # the nadir mount maps to body right.
        pixel = (INTRINSICS.cx + INTRINSICS.fx, INTRINSICS.cy)
        position = locate_target_frd(pixel, INTRINSICS, HEIGHT_M, 0.0, 0.0)
        assert position == pytest.approx([0.0, HEIGHT_M, HEIGHT_M])

    def test_image_down_maps_to_body_aft(self):
        pixel = (INTRINSICS.cx, INTRINSICS.cy + INTRINSICS.fy)
        position = locate_target_frd(pixel, INTRINSICS, HEIGHT_M, 0.0, 0.0)
        assert position == pytest.approx([-HEIGHT_M, 0.0, HEIGHT_M])

    @pytest.mark.parametrize("pitch_rad", [-0.6, -0.2, 0.0, 0.2, 0.6])
    def test_pitch_stretches_slant_range_by_secant(self, pitch_rad):
        # The centre ray of a nadir camera is body-down whatever the attitude, so
        # pitching only changes how far along that ray the plane sits.
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, pitch_rad)
        assert position == pytest.approx([0.0, 0.0, HEIGHT_M / math.cos(pitch_rad)])

    @pytest.mark.parametrize("roll_rad", [-0.6, 0.0, 0.4])
    def test_roll_stretches_slant_range_by_secant(self, roll_rad):
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, roll_rad, 0.0)
        assert position == pytest.approx([0.0, 0.0, HEIGHT_M / math.cos(roll_rad)])

    def test_result_lies_on_the_plane(self):
        roll_rad, pitch_rad = 0.25, -0.15
        pixel = (INTRINSICS.cx + 90.0, INTRINSICS.cy - 40.0)
        position = locate_target_frd(pixel, INTRINSICS, HEIGHT_M, roll_rad, pitch_rad)
        normal = plane_normal_frd(roll_rad, pitch_rad)
        assert float(normal @ position) == pytest.approx(HEIGHT_M)


class TestMount:
    def test_lever_arm_offsets_the_result(self):
        mount = CameraMount(
            rotation=NADIR_DOWN_MOUNT.rotation, translation=np.array([0.3, -0.1, 0.2])
        )
        position = locate_target_frd(
            CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, mount=mount
        )
        assert position == pytest.approx([0.3, -0.1, HEIGHT_M + 0.2])

    def test_forward_facing_mount_sees_targets_ahead(self):
        # Optical axis along body +X, image right -> body right, image down -> body down.
        forward_mount = CameraMount(
            rotation=np.array(
                [
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                ]
            )
        )
        # A pixel below centre looks down-forward at 45 degrees.
        pixel = (INTRINSICS.cx, INTRINSICS.cy + INTRINSICS.fy)
        position = locate_target_frd(
            CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, mount=forward_mount
        )
        # The centre ray is horizontal: parallel to the plane, so unlocalizable.
        assert position is None

        position = locate_target_frd(
            pixel, INTRINSICS, HEIGHT_M, 0.0, 0.0, mount=forward_mount
        )
        assert position == pytest.approx([HEIGHT_M, 0.0, HEIGHT_M])

    def test_mount_does_not_freeze_the_callers_array(self):
        rotation = np.eye(3)
        CameraMount(rotation=rotation)
        rotation[0, 0] = 2.0  # Must not raise.

    def test_rejects_non_orthonormal_rotation(self):
        mount = CameraMount(rotation=np.diag([1.0, 1.0, 2.0]))
        with pytest.raises(ValueError, match="orthonormal"):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, mount=mount)

    def test_rejects_left_handed_rotation(self):
        mount = CameraMount(rotation=np.diag([1.0, 1.0, -1.0]))
        with pytest.raises(ValueError, match="determinant"):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, mount=mount)


class TestRoundTrip:
    """Project known points to pixels and localize them back."""

    @pytest.mark.parametrize("roll_rad", [-0.35, 0.0, 0.45])
    @pytest.mark.parametrize("pitch_rad", [-0.35, 0.0, 0.45])
    @pytest.mark.parametrize("mount_yaw_rad", [0.0, 0.6])
    def test_recovers_points_on_the_plane(self, roll_rad, pitch_rad, mount_yaw_rad):
        mount = CameraMount(
            rotation=NADIR_DOWN_MOUNT.rotation
            @ rotation_zyx(mount_yaw_rad, 0.1, -0.05),
            translation=np.array([0.12, -0.04, 0.31]),
        )
        normal = plane_normal_frd(roll_rad, pitch_rad)

        rng = np.random.default_rng(seed=20260801)
        recovered_any = False
        for _ in range(50):
            # A point on the plane: start below the camera, slide along the plane.
            offset = rng.uniform(-4.0, 4.0, size=3)
            point = mount.translation + HEIGHT_M * normal
            point = point + (offset - float(normal @ offset) * normal)
            assert float(normal @ (point - mount.translation)) == pytest.approx(
                HEIGHT_M
            )

            pixel = project_target_frd(point, INTRINSICS, mount=mount)
            if pixel is None:
                continue

            result = locate_target_frd(
                pixel, INTRINSICS, HEIGHT_M, roll_rad, pitch_rad, mount=mount
            )
            if result is None:
                continue

            recovered_any = True
            assert result == pytest.approx(point, abs=1e-9)

        assert recovered_any, "No sample points survived projection."

    def test_projection_rejects_points_behind_the_camera(self):
        # Above a nadir-down camera is behind it.
        assert project_target_frd([0.0, 0.0, -5.0], INTRINSICS) is None


class TestRejections:
    def test_grazing_ray_is_rejected(self):
        # Far off-axis in image x tips the ray towards the horizontal.
        pixel = (INTRINSICS.cx + 100.0 * INTRINSICS.fx, INTRINSICS.cy)
        assert locate_target_frd(pixel, INTRINSICS, HEIGHT_M, 0.0, 0.0) is None

    def test_ray_pointing_away_from_the_plane_is_rejected(self):
        # Rolled past vertical, the ground plane is no longer below the camera.
        assert (
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, math.pi, 0.0) is None
        )

    def test_incidence_threshold_is_honoured(self):
        # A 45 degree ray passes the default but fails a 30 degree limit.
        pixel = (INTRINSICS.cx + INTRINSICS.fx, INTRINSICS.cy)
        assert locate_target_frd(pixel, INTRINSICS, HEIGHT_M, 0.0, 0.0) is not None
        assert (
            locate_target_frd(
                pixel,
                INTRINSICS,
                HEIGHT_M,
                0.0,
                0.0,
                max_incidence_angle_rad=math.radians(30.0),
            )
            is None
        )

    def test_max_range_rejects_distant_intersections(self):
        assert (
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, max_range_m=HEIGHT_M / 2.0
            )
            is None
        )
        assert (
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, max_range_m=HEIGHT_M * 2.0
            )
            is not None
        )


class TestInputValidation:
    @pytest.mark.parametrize("distance_m", [0.0, -1.0, float("nan"), float("inf")])
    def test_rejects_bad_distance(self, distance_m):
        with pytest.raises(ValueError, match="Distance to plane"):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, distance_m, 0.0, 0.0)

    @pytest.mark.parametrize(
        "intrinsics",
        [
            CameraIntrinsics(fx=0.0, fy=600.0, cx=320.0, cy=240.0),
            CameraIntrinsics(fx=-600.0, fy=600.0, cx=320.0, cy=240.0),
            CameraIntrinsics(fx=600.0, fy=0.0, cx=320.0, cy=240.0),
        ],
    )
    def test_rejects_bad_focal_lengths(self, intrinsics):
        with pytest.raises(ValueError, match="Focal lengths"):
            locate_target_frd(CENTRE_PIXEL, intrinsics, HEIGHT_M, 0.0, 0.0)

    def test_rejects_non_finite_intrinsics(self):
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=float("nan"), cy=240.0)
        with pytest.raises(ValueError, match="must be finite"):
            locate_target_frd(CENTRE_PIXEL, intrinsics, HEIGHT_M, 0.0, 0.0)

    def test_rejects_non_finite_pixel(self):
        with pytest.raises(ValueError, match="Pixel must be finite"):
            locate_target_frd((float("nan"), 240.0), INTRINSICS, HEIGHT_M, 0.0, 0.0)

    def test_rejects_wrong_pixel_shape(self):
        with pytest.raises(ValueError, match="2 components"):
            locate_target_frd((1.0, 2.0, 3.0), INTRINSICS, HEIGHT_M, 0.0, 0.0)

    def test_rejects_non_finite_attitude(self):
        with pytest.raises(ValueError):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, float("inf"))

    @pytest.mark.parametrize("angle_rad", [0.0, -0.1, math.pi / 2.0, math.pi])
    def test_rejects_bad_incidence_limit(self, angle_rad):
        with pytest.raises(ValueError, match="Max incidence angle"):
            locate_target_frd(
                CENTRE_PIXEL,
                INTRINSICS,
                HEIGHT_M,
                0.0,
                0.0,
                max_incidence_angle_rad=angle_rad,
            )

    @pytest.mark.parametrize("max_range_m", [0.0, -5.0, float("nan")])
    def test_rejects_bad_max_range(self, max_range_m):
        with pytest.raises(ValueError, match="Max range"):
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, 0.0, 0.0, max_range_m=max_range_m
            )


class TestIntrinsicsFromFov:
    def test_matches_hand_computed_focal_lengths(self):
        intrinsics = CameraIntrinsics.from_fov(
            640, 480, math.radians(90.0), math.radians(90.0)
        )
        assert intrinsics.fx == pytest.approx(320.0)
        assert intrinsics.fy == pytest.approx(240.0)
        assert intrinsics.cx == pytest.approx(320.0)
        assert intrinsics.cy == pytest.approx(240.0)

    def test_edge_pixel_sits_at_half_the_fov(self):
        hfov_rad = math.radians(80.0)
        intrinsics = CameraIntrinsics.from_fov(640, 480, hfov_rad, math.radians(55.0))
        position = locate_target_frd((640.0, 240.0), intrinsics, HEIGHT_M, 0.0, 0.0)
        # Right edge of the image, so half the horizontal FOV off nadir.
        assert position[1] == pytest.approx(HEIGHT_M * math.tan(hfov_rad / 2.0))

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"image_width_px": 0},
            {"image_height_px": -1},
            {"horizontal_fov_rad": 0.0},
            {"vertical_fov_rad": math.pi},
        ],
    )
    def test_rejects_bad_fov_specs(self, kwargs):
        defaults = {
            "image_width_px": 640,
            "image_height_px": 480,
            "horizontal_fov_rad": math.radians(80.0),
            "vertical_fov_rad": math.radians(55.0),
        }
        defaults.update(kwargs)
        with pytest.raises(ValueError):
            CameraIntrinsics.from_fov(**defaults)
