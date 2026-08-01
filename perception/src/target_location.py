"""Single-frame target localization: image pixel -> position relative to the drone.

Given where a target appears in a rectified camera image, the perpendicular distance
from the camera to the plane the target sits on, and the drone's attitude, this module
returns the target's 3D position in the drone's Forward-Right-Down (FRD) body frame.

Frames
------
Camera optical frame (OpenCV convention):
    +X along image ``+u`` (right), +Y along image ``+v`` (down),
    +Z along the optical axis, out of the lens.
FRD body frame:
    +X forward, +Y right, +Z down. All outputs are in this frame, in metres.
NED world frame:
    Used only to define which way gravity points.

Assumptions
-----------
- The image is **rectified**: lens distortion has already been removed, so no distortion
  coefficients are needed. The intrinsics passed in must be those of the *rectified*
  image (OpenCV's ``P`` matrix / ``camera_info``), not the raw sensor's.
- The target lies on a locally flat, level plane, so the plane's normal is world-down.
- All inputs describe the same instant. Synchronizing the detection, the range
  measurement and the attitude is the caller's job.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

DEFAULT_MAX_INCIDENCE_ANGLE_RAD = math.radians(85.0)

# Tolerance for verifying that a supplied mount rotation is orthonormal.
_ROTATION_TOLERANCE = 1e-6


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics of the rectified image, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_fov(
        cls,
        image_width_px: int,
        image_height_px: int,
        horizontal_fov_rad: float,
        vertical_fov_rad: float,
    ) -> CameraIntrinsics:
        """Derive intrinsics from a field-of-view spec.

        A lossier parameterization than a calibrated intrinsic matrix: it assumes the
        principal point sits exactly at the image centre. Prefer real calibration data
        when it is available.
        """
        if image_width_px <= 0 or image_height_px <= 0:
            raise ValueError("Image dimensions must be positive.")
        if not 0.0 < horizontal_fov_rad < math.pi:
            raise ValueError("Horizontal FOV must be in (0, pi) radians.")
        if not 0.0 < vertical_fov_rad < math.pi:
            raise ValueError("Vertical FOV must be in (0, pi) radians.")

        return cls(
            fx=(image_width_px / 2.0) / math.tan(horizontal_fov_rad / 2.0),
            fy=(image_height_px / 2.0) / math.tan(vertical_fov_rad / 2.0),
            cx=image_width_px / 2.0,
            cy=image_height_px / 2.0,
        )

    def validate(self) -> None:
        """Raise ``ValueError`` if these intrinsics cannot back-project a pixel."""
        for name, value in (
            ("fx", self.fx),
            ("fy", self.fy),
            ("cx", self.cx),
            ("cy", self.cy),
        ):
            if not math.isfinite(value):
                raise ValueError(f"Intrinsic {name} must be finite, got {value}.")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError(
                f"Focal lengths must be positive, got fx={self.fx}, fy={self.fy}."
            )


def _nadir_down_rotation() -> np.ndarray:
    """Camera->FRD rotation for a camera bolted looking straight down.

    Image right maps to body right, image down (``+v``) maps to body aft, and the
    optical axis maps to body down. Columns are the images of the camera basis vectors.
    """
    return np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


@dataclass(frozen=True, eq=False)
class CameraMount:
    """Where the camera sits on the airframe.

    Attributes:
        rotation: 3x3 rotation taking a vector from the camera optical frame to FRD.
        translation: Camera optical centre expressed in FRD, in metres (the lever arm
            from the drone's body origin).
    """

    rotation: np.ndarray = field(default_factory=_nadir_down_rotation)
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self) -> None:
        # Copy rather than view: these are frozen, and we must not freeze a caller's
        # array out from under them.
        rotation = np.array(self.rotation, dtype=float).reshape(3, 3)
        translation = np.array(self.translation, dtype=float).reshape(3)
        rotation.flags.writeable = False
        translation.flags.writeable = False
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "translation", translation)

    def validate(self) -> None:
        """Raise ``ValueError`` if this mount is not a rigid transform."""
        if not np.all(np.isfinite(self.rotation)) or not np.all(
            np.isfinite(self.translation)
        ):
            raise ValueError("Camera mount contains non-finite values.")
        if not np.allclose(
            self.rotation.T @ self.rotation, np.eye(3), atol=_ROTATION_TOLERANCE
        ):
            raise ValueError("Camera mount rotation is not orthonormal.")
        if not math.isclose(
            float(np.linalg.det(self.rotation)), 1.0, abs_tol=_ROTATION_TOLERANCE
        ):
            raise ValueError(
                "Camera mount rotation must have determinant +1 (right-handed)."
            )


NADIR_DOWN_MOUNT = CameraMount()


def plane_normal_frd(roll_rad: float, pitch_rad: float) -> np.ndarray:
    """Unit normal of a level ground plane, expressed in the FRD body frame.

    The plane's normal is world-down, so this is the third row of the body->NED
    rotation for ZYX (yaw-pitch-roll) Tait-Bryan angles. Yaw drops out entirely: a
    level plane looks the same from any heading.

    Points away from the drone when the drone is above the plane, i.e. it is level with
    the drone's own notion of "down".
    """
    if not math.isfinite(roll_rad) or not math.isfinite(pitch_rad):
        raise ValueError("Roll and pitch must be finite.")

    cos_roll, sin_roll = math.cos(roll_rad), math.sin(roll_rad)
    cos_pitch, sin_pitch = math.cos(pitch_rad), math.sin(pitch_rad)

    return np.array(
        [
            -sin_pitch,
            sin_roll * cos_pitch,
            cos_roll * cos_pitch,
        ]
    )


def locate_target_frd(
    pixel: Sequence[float],
    intrinsics: CameraIntrinsics,
    distance_to_plane_m: float,
    roll_rad: float,
    pitch_rad: float,
    mount: CameraMount = NADIR_DOWN_MOUNT,
    max_incidence_angle_rad: float = DEFAULT_MAX_INCIDENCE_ANGLE_RAD,
    max_range_m: float | None = None,
) -> np.ndarray | None:
    """Locate a detected target relative to the drone.

    Back-projects the pixel into a ray, rotates it into FRD, and intersects it with the
    ground plane implied by the drone's attitude and the measured distance.

    Args:
        pixel: Target's ``(u, v)`` location in the rectified image, in pixels.
        intrinsics: Intrinsics of that same rectified image.
        distance_to_plane_m: Perpendicular distance from the camera optical centre to
            the target's plane, in metres. This is *not* a raw rangefinder reading: a
            downward rangefinder on a drone pitched by ``theta`` reads
            ``distance / cos(theta)``.
        roll_rad: Drone roll, radians, ZYX Tait-Bryan.
        pitch_rad: Drone pitch, radians, ZYX Tait-Bryan. Yaw is not needed — the answer
            is in the body frame and the plane is level.
        mount: Camera pose on the airframe. Defaults to a nadir-down mount at the body
            origin.
        max_incidence_angle_rad: Reject rays striking the plane at a shallower angle
            than this, measured from the plane normal.
        max_range_m: If given, reject intersections further than this along the ray.

    Returns:
        ``(forward, right, down)`` in metres relative to the drone's body origin, or
        ``None`` if the detection cannot be reliably localized.

    Raises:
        ValueError: If any input is malformed — bad intrinsics, a non-positive or
            non-finite distance, a non-finite pixel, or a mount that is not a rigid
            transform. These are caller bugs, not runtime conditions.

    Note:
        Position error grows quadratically with obliquity. A one-pixel detection error
        displaces the result by roughly ``s^2 / (f * h)`` metres, where ``s`` is the
        slant range and ``h`` the perpendicular distance — which is why shallow
        incidence angles are rejected outright rather than reported with low confidence.
    """
    intrinsics.validate()
    mount.validate()

    pixel_array = np.asarray(pixel, dtype=float).reshape(-1)
    if pixel_array.size != 2:
        raise ValueError(f"Pixel must have 2 components, got {pixel_array.size}.")
    if not np.all(np.isfinite(pixel_array)):
        raise ValueError(f"Pixel must be finite, got {pixel}.")

    if not math.isfinite(distance_to_plane_m) or distance_to_plane_m <= 0.0:
        raise ValueError(
            f"Distance to plane must be positive and finite, got {distance_to_plane_m}."
        )
    if not math.isfinite(max_incidence_angle_rad) or not (
        0.0 < max_incidence_angle_rad < math.pi / 2.0
    ):
        raise ValueError(
            "Max incidence angle must be in (0, pi/2) radians, got "
            f"{max_incidence_angle_rad}."
        )
    if max_range_m is not None and (
        not math.isfinite(max_range_m) or max_range_m <= 0.0
    ):
        raise ValueError(f"Max range must be positive and finite, got {max_range_m}.")

    # Back-project the pixel into a ray in the camera optical frame.
    u, v = pixel_array
    direction_cam = np.array(
        [
            (u - intrinsics.cx) / intrinsics.fx,
            (v - intrinsics.cy) / intrinsics.fy,
            1.0,
        ]
    )

    direction_frd = mount.rotation @ direction_cam
    direction_frd /= np.linalg.norm(direction_frd)

    normal_frd = plane_normal_frd(roll_rad, pitch_rad)

    # How squarely the ray meets the plane. Non-positive means the ray points away from
    # the plane entirely (target behind the camera); small-positive means a grazing hit
    # whose intersection is too error-sensitive to trust.
    incidence_cosine = float(normal_frd @ direction_frd)
    if incidence_cosine <= math.cos(max_incidence_angle_rad):
        return None

    slant_range_m = distance_to_plane_m / incidence_cosine
    if max_range_m is not None and slant_range_m > max_range_m:
        return None

    return mount.translation + slant_range_m * direction_frd


def project_target_frd(
    point_frd: Sequence[float],
    intrinsics: CameraIntrinsics,
    mount: CameraMount = NADIR_DOWN_MOUNT,
) -> tuple[float, float] | None:
    """Forward model: project an FRD point back into rectified image pixels.

    The inverse of :func:`locate_target_frd`, useful for testing and for predicting
    where a known target should appear.

    Returns:
        The ``(u, v)`` pixel, or ``None`` if the point lies behind the camera.
    """
    intrinsics.validate()
    mount.validate()

    point_array = np.asarray(point_frd, dtype=float).reshape(-1)
    if point_array.size != 3:
        raise ValueError(f"Point must have 3 components, got {point_array.size}.")
    if not np.all(np.isfinite(point_array)):
        raise ValueError(f"Point must be finite, got {point_frd}.")

    point_cam = mount.rotation.T @ (point_array - mount.translation)
    if point_cam[2] <= 0.0:
        return None

    return (
        intrinsics.cx + intrinsics.fx * point_cam[0] / point_cam[2],
        intrinsics.cy + intrinsics.fy * point_cam[1] / point_cam[2],
    )
