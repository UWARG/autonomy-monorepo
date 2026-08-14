"""Tests for single-frame target localization."""

import math

import numpy as np
import pytest

from target_location import (
    NADIR_DOWN_MOUNT,
    CameraIntrinsics,
    CameraMount,
    GroundPlaneGeometry,
    locate_target_frd,
    plane_normal_frd,
    project_target_frd,
)
from utils import Attitude, ImageFrame, TargetPosition

INTRINSICS = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
CENTRE_PIXEL = ImageFrame(u=INTRINSICS.cx, v=INTRINSICS.cy)
HEIGHT_M = 10.0
LEVEL = GroundPlaneGeometry.level()

# ENU <- NED and FLU <- FRD axis swaps, used to cross-check the MAVROS constructor.
NED_FROM_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_FROM_FRD = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def rotation_zyx(yaw_rad: float, pitch_rad: float, roll_rad: float) -> np.ndarray:
    """Body->NED rotation for ZYX Tait-Bryan angles."""
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)

    r_z = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
    r_y = np.array([[cos_p, 0.0, sin_p], [0.0, 1.0, 0.0], [-sin_p, 0.0, cos_p]])
    r_x = np.array([[1.0, 0.0, 0.0], [0.0, cos_r, -sin_r], [0.0, sin_r, cos_r]])

    return r_z @ r_y @ r_x


def quaternion_from_zyx(yaw_rad: float, pitch_rad: float, roll_rad: float):
    """(w, x, y, z) for the same ZYX Tait-Bryan angles as ``rotation_zyx``."""
    cos_y, sin_y = math.cos(yaw_rad / 2.0), math.sin(yaw_rad / 2.0)
    cos_p, sin_p = math.cos(pitch_rad / 2.0), math.sin(pitch_rad / 2.0)
    cos_r, sin_r = math.cos(roll_rad / 2.0), math.sin(roll_rad / 2.0)

    return (
        cos_r * cos_p * cos_y + sin_r * sin_p * sin_y,
        sin_r * cos_p * cos_y - cos_r * sin_p * sin_y,
        cos_r * sin_p * cos_y + sin_r * cos_p * sin_y,
        cos_r * cos_p * sin_y - sin_r * sin_p * cos_y,
    )


def quaternion_from_axis_angle(axis, angle_rad: float):
    """(w, x, y, z) rotating by ``angle_rad`` about ``axis``."""
    unit_axis = np.asarray(axis, dtype=float)
    unit_axis = unit_axis / np.linalg.norm(unit_axis)
    sin_half = math.sin(angle_rad / 2.0)

    return (
        math.cos(angle_rad / 2.0),
        unit_axis[0] * sin_half,
        unit_axis[1] * sin_half,
        unit_axis[2] * sin_half,
    )


def matrix_from_quaternion(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix for a unit quaternion (w, x, y, z)."""
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    )


class TestImageFrame:
    def test_accepts_a_bare_tuple(self):
        from_frame = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL)
        from_tuple = locate_target_frd(
            (INTRINSICS.cx, INTRINSICS.cy), INTRINSICS, HEIGHT_M, LEVEL
        )
        assert from_tuple.to_array() == pytest.approx(from_frame.to_array())

    def test_accepts_a_numpy_pair(self):
        pixel = np.array([INTRINSICS.cx, INTRINSICS.cy])
        assert locate_target_frd(pixel, INTRINSICS, HEIGHT_M, LEVEL) is not None

    def test_to_tuple_round_trips(self):
        assert ImageFrame(u=1.5, v=-2.5).to_tuple() == (1.5, -2.5)


class TestGroundPlaneGeometry:
    def test_level_is_body_down(self):
        assert GroundPlaneGeometry.level().down_frd == pytest.approx([0.0, 0.0, 1.0])

    def test_plane_normal_is_the_down_vector(self):
        attitude = GroundPlaneGeometry.from_euler(0.3, -0.2)
        assert attitude.plane_normal_frd == pytest.approx(attitude.down_frd)

    def test_down_vector_is_normalized_on_construction(self):
        attitude = GroundPlaneGeometry(down_frd=np.array([0.0, 0.0, 7.0]))
        assert attitude.down_frd == pytest.approx([0.0, 0.0, 1.0])

    @pytest.mark.parametrize("roll_rad", [-0.6, 0.0, 0.4])
    @pytest.mark.parametrize("pitch_rad", [-0.6, 0.0, 0.4])
    def test_from_attitude_matches_from_euler(self, roll_rad, pitch_rad):
        attitude = Attitude(
            roll=roll_rad,
            pitch=pitch_rad,
            yaw=1.3,
            rollspeed=0.1,
            pitchspeed=-0.2,
            yawspeed=0.3,
        )
        assert GroundPlaneGeometry.from_attitude(attitude).down_frd == pytest.approx(
            GroundPlaneGeometry.from_euler(roll_rad, pitch_rad).down_frd
        )

    @pytest.mark.parametrize("roll_rad", [-1.0, -0.3, 0.0, 0.3, 1.0])
    @pytest.mark.parametrize("pitch_rad", [-1.0, -0.3, 0.0, 0.3, 1.0])
    def test_normal_is_always_a_unit_vector(self, roll_rad, pitch_rad):
        normal = GroundPlaneGeometry.from_euler(roll_rad, pitch_rad).plane_normal_frd
        assert np.linalg.norm(normal) == pytest.approx(1.0)

    @pytest.mark.parametrize("roll_rad", [-0.4, 0.0, 0.7])
    @pytest.mark.parametrize("pitch_rad", [-0.4, 0.0, 0.7])
    @pytest.mark.parametrize("yaw_rad", [-2.0, 0.0, 1.1])
    def test_matches_third_row_of_body_to_ned_rotation(
        self, roll_rad, pitch_rad, yaw_rad
    ):
        expected = rotation_zyx(yaw_rad, pitch_rad, roll_rad)[2, :]
        actual = GroundPlaneGeometry.from_euler(roll_rad, pitch_rad, yaw_rad).down_frd
        assert actual == pytest.approx(expected)

    @pytest.mark.parametrize("yaw_rad", [-3.0, -1.0, 0.0, 2.5])
    def test_yaw_is_accepted_and_ignored(self, yaw_rad):
        expected = GroundPlaneGeometry.from_euler(0.3, -0.2).down_frd
        actual = GroundPlaneGeometry.from_euler(0.3, -0.2, yaw_rad).down_frd
        assert actual == pytest.approx(expected)

    def test_nose_up_tilts_gravity_aft(self):
        # Pitched nose up, "down" is behind the drone: negative forward component.
        assert GroundPlaneGeometry.from_euler(0.0, 0.5).down_frd[0] < 0.0

    @pytest.mark.parametrize("roll_rad", [-2.5, -0.4, 0.0, 0.7, 2.5])
    @pytest.mark.parametrize("pitch_rad", [-1.2, -0.4, 0.0, 0.7, 1.2])
    def test_roll_and_pitch_properties_recover_the_inputs(self, roll_rad, pitch_rad):
        attitude = GroundPlaneGeometry.from_euler(roll_rad, pitch_rad)
        assert attitude.roll_rad == pytest.approx(roll_rad)
        assert attitude.pitch_rad == pytest.approx(pitch_rad)

    @pytest.mark.parametrize("angles", [(0.0, 0.0), (0.4, -0.3), (-1.1, 0.9)])
    @pytest.mark.parametrize("yaw_rad", [0.0, 1.7])
    def test_quaternion_agrees_with_euler(self, angles, yaw_rad):
        roll_rad, pitch_rad = angles
        quaternion = quaternion_from_zyx(yaw_rad, pitch_rad, roll_rad)
        expected = GroundPlaneGeometry.from_euler(roll_rad, pitch_rad).down_frd
        actual = GroundPlaneGeometry.from_quaternion(*quaternion).down_frd
        assert actual == pytest.approx(expected)

    def test_quaternion_survives_gimbal_lock(self):
        # Straight-down pitch is where an Euler round trip would degenerate.
        quaternion = quaternion_from_zyx(0.9, math.pi / 2.0, 0.0)
        down_frd = GroundPlaneGeometry.from_quaternion(*quaternion).down_frd
        assert down_frd == pytest.approx([-1.0, 0.0, 0.0], abs=1e-12)

    def test_quaternion_is_normalized_on_construction(self):
        down_frd = GroundPlaneGeometry.from_quaternion(2.0, 0.0, 0.0, 0.0).down_frd
        assert down_frd == pytest.approx([0.0, 0.0, 1.0])

    @pytest.mark.parametrize(
        "axis", [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, -2.0, 0.5)]
    )
    @pytest.mark.parametrize("angle_rad", [-1.3, 0.0, 0.6, 2.2])
    def test_mavros_quaternion_matches_the_enu_flu_derivation(self, axis, angle_rad):
        # MAVROS publishes body-FLU -> ENU. Convert that rotation to body-FRD -> NED
        # the long way and compare against the constructor's closed form.
        quaternion = quaternion_from_axis_angle(axis, angle_rad)
        enu_from_flu = matrix_from_quaternion(*quaternion)
        ned_from_frd = NED_FROM_ENU @ enu_from_flu @ FLU_FROM_FRD

        actual = GroundPlaneGeometry.from_mavros_quaternion(*quaternion).down_frd
        assert actual == pytest.approx(ned_from_frd[2, :])

    def test_mavros_identity_quaternion_is_level(self):
        level = GroundPlaneGeometry.from_mavros_quaternion(1.0, 0.0, 0.0, 0.0)
        assert level.down_frd == pytest.approx([0.0, 0.0, 1.0])

    def test_mavros_and_ned_conventions_disagree_on_forward(self):
        # The trap the separate constructor exists to prevent: same numbers, opposite
        # forward axis. A target ahead of the drone would be reported behind it.
        quaternion = quaternion_from_axis_angle((0.0, 1.0, 0.0), 0.4)
        ned = GroundPlaneGeometry.from_quaternion(*quaternion).down_frd
        mavros = GroundPlaneGeometry.from_mavros_quaternion(*quaternion).down_frd

        assert ned[0] == pytest.approx(-mavros[0])
        assert ned[0] != pytest.approx(0.0)

    @pytest.mark.parametrize(
        "kwargs", [{"roll_rad": float("nan")}, {"pitch_rad": float("inf")}]
    )
    def test_rejects_non_finite_euler(self, kwargs):
        angles = {"roll_rad": 0.0, "pitch_rad": 0.0}
        angles.update(kwargs)
        with pytest.raises(ValueError, match="must be finite"):
            GroundPlaneGeometry.from_euler(**angles)

    def test_rejects_non_finite_yaw(self):
        with pytest.raises(ValueError, match="yaw must be finite"):
            GroundPlaneGeometry.from_euler(0.0, 0.0, float("nan"))

    def test_rejects_zero_quaternion(self):
        with pytest.raises(ValueError, match="non-zero norm"):
            GroundPlaneGeometry.from_quaternion(0.0, 0.0, 0.0, 0.0)

    def test_rejects_non_finite_quaternion(self):
        with pytest.raises(ValueError, match="must be finite"):
            GroundPlaneGeometry.from_quaternion(float("nan"), 0.0, 0.0, 0.0)

    def test_rejects_zero_down_vector(self):
        with pytest.raises(ValueError, match="non-zero length"):
            GroundPlaneGeometry(down_frd=np.zeros(3))

    def test_free_function_matches_the_class(self):
        assert plane_normal_frd(0.3, -0.2) == pytest.approx(
            GroundPlaneGeometry.from_euler(0.3, -0.2).plane_normal_frd
        )


class TestTargetPosition:
    def test_to_array_is_forward_right_down(self):
        position = TargetPosition(forward_m=1.0, right_m=2.0, down_m=3.0)
        assert position.to_array() == pytest.approx([1.0, 2.0, 3.0])

    def test_range_is_the_euclidean_norm(self):
        position = TargetPosition(forward_m=3.0, right_m=0.0, down_m=4.0)
        assert position.range_m == pytest.approx(5.0)


class TestNadirGeometry:
    def test_centre_pixel_level_drone_is_directly_below(self):
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL)
        assert position.to_array() == pytest.approx([0.0, 0.0, HEIGHT_M])

    def test_forty_five_degree_pixel_lands_one_height_to_the_right(self):
        # u offset by exactly fx is a 45 degree ray in the image's x direction, which
        # the nadir mount maps to body right.
        pixel = ImageFrame(u=INTRINSICS.cx + INTRINSICS.fx, v=INTRINSICS.cy)
        position = locate_target_frd(pixel, INTRINSICS, HEIGHT_M, LEVEL)
        assert position.to_array() == pytest.approx([0.0, HEIGHT_M, HEIGHT_M])

    def test_image_down_maps_to_body_aft(self):
        pixel = ImageFrame(u=INTRINSICS.cx, v=INTRINSICS.cy + INTRINSICS.fy)
        position = locate_target_frd(pixel, INTRINSICS, HEIGHT_M, LEVEL)
        assert position.to_array() == pytest.approx([-HEIGHT_M, 0.0, HEIGHT_M])

    @pytest.mark.parametrize("pitch_rad", [-0.6, -0.2, 0.0, 0.2, 0.6])
    def test_pitch_stretches_slant_range_by_secant(self, pitch_rad):
        # The centre ray of a nadir camera is body-down whatever the attitude, so
        # pitching only changes how far along that ray the plane sits.
        attitude = GroundPlaneGeometry.from_euler(0.0, pitch_rad)
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, attitude)
        assert position.to_array() == pytest.approx(
            [0.0, 0.0, HEIGHT_M / math.cos(pitch_rad)]
        )

    @pytest.mark.parametrize("roll_rad", [-0.6, 0.0, 0.4])
    def test_roll_stretches_slant_range_by_secant(self, roll_rad):
        attitude = GroundPlaneGeometry.from_euler(roll_rad, 0.0)
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, attitude)
        assert position.to_array() == pytest.approx(
            [0.0, 0.0, HEIGHT_M / math.cos(roll_rad)]
        )

    @pytest.mark.parametrize("roll_rad", [-0.3, 0.0, 0.45])
    @pytest.mark.parametrize("pitch_rad", [-0.3, 0.0, 0.45])
    def test_accepts_a_shared_attitude_directly(self, roll_rad, pitch_rad):
        # Callers holding a MAVLink attitude should not have to convert it by hand.
        attitude = Attitude(
            roll=roll_rad,
            pitch=pitch_rad,
            yaw=-0.8,
            rollspeed=0.0,
            pitchspeed=0.0,
            yawspeed=0.0,
        )
        pixel = ImageFrame(u=INTRINSICS.cx + 55.0, v=INTRINSICS.cy - 25.0)
        assert locate_target_frd(
            pixel, INTRINSICS, HEIGHT_M, attitude
        ).to_array() == pytest.approx(
            locate_target_frd(
                pixel,
                INTRINSICS,
                HEIGHT_M,
                GroundPlaneGeometry.from_euler(roll_rad, pitch_rad),
            ).to_array()
        )

    def test_result_lies_on_the_plane(self):
        attitude = GroundPlaneGeometry.from_euler(0.25, -0.15)
        pixel = ImageFrame(u=INTRINSICS.cx + 90.0, v=INTRINSICS.cy - 40.0)
        position = locate_target_frd(pixel, INTRINSICS, HEIGHT_M, attitude)
        assert float(attitude.plane_normal_frd @ position.to_array()) == pytest.approx(
            HEIGHT_M
        )


class TestMount:
    def test_lever_arm_offsets_the_result(self):
        mount = CameraMount(
            rotation=NADIR_DOWN_MOUNT.rotation, translation=np.array([0.3, -0.1, 0.2])
        )
        position = locate_target_frd(
            CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, mount=mount
        )
        assert position.to_array() == pytest.approx([0.3, -0.1, HEIGHT_M + 0.2])

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
        # The centre ray is horizontal: parallel to the plane, so unlocalizable.
        assert (
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, mount=forward_mount
            )
            is None
        )

        # A pixel below centre looks down-forward at 45 degrees.
        pixel = ImageFrame(u=INTRINSICS.cx, v=INTRINSICS.cy + INTRINSICS.fy)
        position = locate_target_frd(
            pixel, INTRINSICS, HEIGHT_M, LEVEL, mount=forward_mount
        )
        assert position.to_array() == pytest.approx([HEIGHT_M, 0.0, HEIGHT_M])

    def test_mount_does_not_freeze_the_callers_array(self):
        rotation = np.eye(3)
        CameraMount(rotation=rotation)
        rotation[0, 0] = 2.0  # Must not raise.

    def test_rejects_non_orthonormal_rotation(self):
        mount = CameraMount(rotation=np.diag([1.0, 1.0, 2.0]))
        with pytest.raises(ValueError, match="orthonormal"):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, mount=mount)

    def test_rejects_left_handed_rotation(self):
        mount = CameraMount(rotation=np.diag([1.0, 1.0, -1.0]))
        with pytest.raises(ValueError, match="determinant"):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, mount=mount)


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
        attitude = GroundPlaneGeometry.from_euler(roll_rad, pitch_rad)
        normal = attitude.plane_normal_frd

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
                pixel, INTRINSICS, HEIGHT_M, attitude, mount=mount
            )
            if result is None:
                continue

            recovered_any = True
            assert result.to_array() == pytest.approx(point, abs=1e-9)

        assert recovered_any, "No sample points survived projection."

    def test_projection_accepts_a_target_position(self):
        position = locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL)
        pixel = project_target_frd(position, INTRINSICS)
        assert pixel.to_tuple() == pytest.approx(CENTRE_PIXEL.to_tuple())

    def test_projection_rejects_points_behind_the_camera(self):
        # Above a nadir-down camera is behind it.
        assert project_target_frd([0.0, 0.0, -5.0], INTRINSICS) is None

    def test_projection_rejects_wrong_point_shape(self):
        with pytest.raises(ValueError, match="3 components"):
            project_target_frd([1.0, 2.0], INTRINSICS)

    def test_projection_rejects_non_finite_points(self):
        with pytest.raises(ValueError, match="must be finite"):
            project_target_frd([0.0, float("nan"), 5.0], INTRINSICS)


class TestRejections:
    def test_grazing_ray_is_rejected(self):
        # Far off-axis in image x tips the ray towards the horizontal.
        pixel = ImageFrame(u=INTRINSICS.cx + 100.0 * INTRINSICS.fx, v=INTRINSICS.cy)
        assert locate_target_frd(pixel, INTRINSICS, HEIGHT_M, LEVEL) is None

    def test_ray_pointing_away_from_the_plane_is_rejected(self):
        # Rolled past vertical, the ground plane is no longer below the camera.
        inverted = GroundPlaneGeometry.from_euler(math.pi, 0.0)
        assert locate_target_frd(CENTRE_PIXEL, INTRINSICS, HEIGHT_M, inverted) is None

    def test_incidence_threshold_is_honoured(self):
        # A 45 degree ray passes the default but fails a 30 degree limit.
        pixel = ImageFrame(u=INTRINSICS.cx + INTRINSICS.fx, v=INTRINSICS.cy)
        assert locate_target_frd(pixel, INTRINSICS, HEIGHT_M, LEVEL) is not None
        assert (
            locate_target_frd(
                pixel,
                INTRINSICS,
                HEIGHT_M,
                LEVEL,
                max_incidence_angle_rad=math.radians(30.0),
            )
            is None
        )

    def test_max_range_rejects_distant_intersections(self):
        assert (
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, max_range_m=HEIGHT_M / 2.0
            )
            is None
        )
        assert (
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, max_range_m=HEIGHT_M * 2.0
            )
            is not None
        )


class TestInputValidation:
    @pytest.mark.parametrize("distance_m", [0.0, -1.0, float("nan"), float("inf")])
    def test_rejects_bad_distance(self, distance_m):
        with pytest.raises(ValueError, match="Distance to plane"):
            locate_target_frd(CENTRE_PIXEL, INTRINSICS, distance_m, LEVEL)

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
            locate_target_frd(CENTRE_PIXEL, intrinsics, HEIGHT_M, LEVEL)

    def test_rejects_non_finite_intrinsics(self):
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=float("nan"), cy=240.0)
        with pytest.raises(ValueError, match="must be finite"):
            locate_target_frd(CENTRE_PIXEL, intrinsics, HEIGHT_M, LEVEL)

    def test_rejects_non_finite_pixel(self):
        with pytest.raises(ValueError, match="Pixel must be finite"):
            locate_target_frd(
                ImageFrame(u=float("nan"), v=240.0), INTRINSICS, HEIGHT_M, LEVEL
            )

    def test_rejects_wrong_pixel_shape(self):
        with pytest.raises(ValueError, match="2 components"):
            locate_target_frd((1.0, 2.0, 3.0), INTRINSICS, HEIGHT_M, LEVEL)

    @pytest.mark.parametrize("angle_rad", [0.0, -0.1, math.pi / 2.0, math.pi])
    def test_rejects_bad_incidence_limit(self, angle_rad):
        with pytest.raises(ValueError, match="Max incidence angle"):
            locate_target_frd(
                CENTRE_PIXEL,
                INTRINSICS,
                HEIGHT_M,
                LEVEL,
                max_incidence_angle_rad=angle_rad,
            )

    @pytest.mark.parametrize("max_range_m", [0.0, -5.0, float("nan")])
    def test_rejects_bad_max_range(self, max_range_m):
        with pytest.raises(ValueError, match="Max range"):
            locate_target_frd(
                CENTRE_PIXEL, INTRINSICS, HEIGHT_M, LEVEL, max_range_m=max_range_m
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
        position = locate_target_frd(
            ImageFrame(u=640.0, v=240.0), intrinsics, HEIGHT_M, LEVEL
        )
        # Right edge of the image, so half the horizontal FOV off nadir.
        assert position.right_m == pytest.approx(HEIGHT_M * math.tan(hfov_rad / 2.0))

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
