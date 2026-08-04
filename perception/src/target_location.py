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
from typing import Union

import numpy as np

DEFAULT_MAX_INCIDENCE_ANGLE_RAD = math.radians(85.0)

# Tolerance for verifying that a supplied mount rotation is orthonormal.
_ROTATION_TOLERANCE = 1e-6

# Below this, a vector is too short for its direction to mean anything.
_MIN_VECTOR_NORM = 1e-9


@dataclass(frozen=True)
class ImageFrame:
    """Where the detector found the target within the image, in pixels.

    "Frame" here is the image's coordinate frame, not a video frame: this module never
    touches pixel data, only the detection's location within it.
    """

    u: float
    v: float

    def to_tuple(self) -> tuple[float, float]:
        return (self.u, self.v)

    def validate(self) -> None:
        """Raise ``ValueError`` if this detection cannot be back-projected."""
        if not math.isfinite(self.u) or not math.isfinite(self.v):
            raise ValueError(f"Pixel must be finite, got (u={self.u}, v={self.v}).")


# Callers holding a bare (u, v) pair should not have to wrap it by hand.
ImageFrameLike = Union[ImageFrame, Sequence[float]]


def _as_image_frame(value: ImageFrameLike) -> ImageFrame:
    if isinstance(value, ImageFrame):
        return value

    components = np.asarray(value, dtype=float).reshape(-1)
    if components.size != 2:
        raise ValueError(f"Pixel must have 2 components, got {components.size}.")
    return ImageFrame(u=float(components[0]), v=float(components[1]))


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


def _normalized_quaternion(w: float, x: float, y: float, z: float) -> np.ndarray:
    components = np.array([w, x, y, z], dtype=float)
    if not np.all(np.isfinite(components)):
        raise ValueError(f"Quaternion must be finite, got {components.tolist()}.")

    norm = float(np.linalg.norm(components))
    if norm < _MIN_VECTOR_NORM:
        raise ValueError("Quaternion must have non-zero norm.")
    return components / norm


@dataclass(frozen=True, eq=False)
class Attitude:
    """The drone's orientation, reduced to the one thing localization needs.

    A level target plane has a world-down normal, so the only part of the drone's
    orientation that affects the answer is *which way is down in the body frame*.
    Heading drops out entirely — the result is expressed in the body frame, and a level
    plane looks identical from every heading.

    Attributes:
        down_frd: Unit vector along world-down, expressed in FRD. Normalized on
            construction.
    """

    down_frd: np.ndarray

    def __post_init__(self) -> None:
        vector = np.array(self.down_frd, dtype=float).reshape(3)
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"Down vector must be finite, got {vector.tolist()}.")

        norm = float(np.linalg.norm(vector))
        if norm < _MIN_VECTOR_NORM:
            raise ValueError("Down vector must have non-zero length.")

        vector = vector / norm
        vector.flags.writeable = False
        object.__setattr__(self, "down_frd", vector)

    @property
    def plane_normal_frd(self) -> np.ndarray:
        """Unit normal of the level target plane, in FRD.

        Identical to :attr:`down_frd` — the plane is assumed level, so its normal *is*
        world-down. Named separately because that is the role it plays in the geometry.
        """
        return self.down_frd

    @property
    def roll_rad(self) -> float:
        """Roll recovered from the down vector, radians. For logging and debugging."""
        return math.atan2(float(self.down_frd[1]), float(self.down_frd[2]))

    @property
    def pitch_rad(self) -> float:
        """Pitch recovered from the down vector, radians. For logging and debugging."""
        return math.asin(float(np.clip(-self.down_frd[0], -1.0, 1.0)))

    @classmethod
    def level(cls) -> Attitude:
        """A perfectly level drone: body-down and world-down coincide."""
        return cls(down_frd=np.array([0.0, 0.0, 1.0]))

    @classmethod
    def from_euler(
        cls, roll_rad: float, pitch_rad: float, yaw_rad: float = 0.0
    ) -> Attitude:
        """Build from ZYX (yaw-pitch-roll) Tait-Bryan angles in a NED world frame.

        ``yaw_rad`` is accepted so call sites holding a full (roll, pitch, yaw) triple
        need not unpack it, but it is **ignored**: it cannot affect the result. Only its
        finiteness is checked.
        """
        for name, value in (
            ("roll", roll_rad),
            ("pitch", pitch_rad),
            ("yaw", yaw_rad),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite, got {value}.")

        cos_roll, sin_roll = math.cos(roll_rad), math.sin(roll_rad)
        cos_pitch, sin_pitch = math.cos(pitch_rad), math.sin(pitch_rad)

        # Third row of the body->NED rotation, i.e. world-down seen from the body.
        return cls(
            down_frd=np.array([-sin_pitch, sin_roll * cos_pitch, cos_roll * cos_pitch])
        )

    @classmethod
    def from_quaternion(cls, w: float, x: float, y: float, z: float) -> Attitude:
        """Build from a quaternion rotating **body-FRD vectors into NED**.

        Avoids Euler angles entirely, so it is immune to gimbal lock at +/-90 degrees
        pitch.

        This is *not* the convention on ``/mavros/imu/data`` — see
        :meth:`from_mavros_quaternion`.
        """
        w, x, y, z = _normalized_quaternion(w, x, y, z)

        # Third row of the rotation matrix: world-down expressed in the body frame.
        return cls(
            down_frd=np.array(
                [
                    2.0 * (x * z - w * y),
                    2.0 * (y * z + w * x),
                    1.0 - 2.0 * (x * x + y * y),
                ]
            )
        )

    @classmethod
    def from_mavros_quaternion(cls, w: float, x: float, y: float, z: float) -> Attitude:
        """Build from a ``sensor_msgs/Imu`` orientation as MAVROS publishes it.

        MAVROS follows the ROS convention: the quaternion rotates **body-FLU** (forward,
        left, up) vectors into an **ENU** world frame, not FRD into NED. Feeding such a
        quaternion to :meth:`from_quaternion` silently flips the forward axis, putting
        targets behind the drone when they are in front — hence a separate constructor
        rather than a documentation note.
        """
        w, x, y, z = _normalized_quaternion(w, x, y, z)

        # World-up in FLU is the third row of the FLU->ENU rotation. Negating it gives
        # world-down, and the FLU->FRD axis flip (y, z negated) leaves only the forward
        # component's sign changed relative to the NED form above.
        return cls(
            down_frd=np.array(
                [
                    -2.0 * (x * z - w * y),
                    2.0 * (y * z + w * x),
                    1.0 - 2.0 * (x * x + y * y),
                ]
            )
        )


@dataclass(frozen=True)
class TargetPosition:
    """Where the target is, relative to the drone's body origin, in metres.

    Position only. A single ray to a detection centroid carries no information about
    how the target is *oriented*, so this deliberately is not a full pose; anything
    claiming otherwise would be fabricating three numbers.
    """

    forward_m: float
    right_m: float
    down_m: float

    def to_array(self) -> np.ndarray:
        """As a ``(forward, right, down)`` numpy array."""
        return np.array([self.forward_m, self.right_m, self.down_m])

    @property
    def range_m(self) -> float:
        """Straight-line distance from the drone's body origin to the target."""
        return float(np.linalg.norm(self.to_array()))


TargetPositionLike = Union[TargetPosition, Sequence[float]]


def _as_position_array(value: TargetPositionLike) -> np.ndarray:
    if isinstance(value, TargetPosition):
        return value.to_array()

    components = np.asarray(value, dtype=float).reshape(-1)
    if components.size != 3:
        raise ValueError(f"Point must have 3 components, got {components.size}.")
    return components


def plane_normal_frd(roll_rad: float, pitch_rad: float) -> np.ndarray:
    """Unit normal of a level ground plane, expressed in the FRD body frame.

    Convenience wrapper over ``Attitude.from_euler(...).plane_normal_frd``.
    """
    return Attitude.from_euler(roll_rad, pitch_rad).plane_normal_frd


def locate_target_frd(
    image_frame: ImageFrameLike,
    intrinsics: CameraIntrinsics,
    distance_to_plane_m: float,
    attitude: Attitude,
    mount: CameraMount = NADIR_DOWN_MOUNT,
    max_incidence_angle_rad: float = DEFAULT_MAX_INCIDENCE_ANGLE_RAD,
    max_range_m: float | None = None,
) -> TargetPosition | None:
    """Locate a detected target relative to the drone.

    Back-projects the detection into a ray, rotates it into FRD, and intersects it with
    the ground plane implied by the drone's attitude and the measured distance.

    Args:
        image_frame: Where the detector found the target, as an :class:`ImageFrame` or
            a bare ``(u, v)`` pair, in pixels of the rectified image.
        intrinsics: Intrinsics of that same rectified image.
        distance_to_plane_m: Perpendicular distance from the camera optical centre to
            the target's plane, in metres. This is *not* a raw rangefinder reading: a
            downward rangefinder on a drone pitched by ``theta`` reads
            ``distance / cos(theta)``.
        attitude: The drone's orientation. Build it with :meth:`Attitude.from_euler`,
            :meth:`Attitude.from_quaternion`, or :meth:`Attitude.from_mavros_quaternion`
            depending on what your source publishes.
        mount: Camera pose on the airframe. Defaults to a nadir-down mount at the body
            origin.
        max_incidence_angle_rad: Reject rays striking the plane at a shallower angle
            than this, measured from the plane normal.
        max_range_m: If given, reject intersections further than this along the ray.

    Returns:
        A :class:`TargetPosition` in metres relative to the drone's body origin, or
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

    detection = _as_image_frame(image_frame)
    detection.validate()

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

    # Back-project the detection into a ray in the camera optical frame.
    direction_cam = np.array(
        [
            (detection.u - intrinsics.cx) / intrinsics.fx,
            (detection.v - intrinsics.cy) / intrinsics.fy,
            1.0,
        ]
    )

    direction_frd = mount.rotation @ direction_cam
    direction_frd /= np.linalg.norm(direction_frd)

    normal_frd = attitude.plane_normal_frd

    # How squarely the ray meets the plane. Non-positive means the ray points away from
    # the plane entirely (target behind the camera); small-positive means a grazing hit
    # whose intersection is too error-sensitive to trust.
    incidence_cosine = float(normal_frd @ direction_frd)
    if incidence_cosine <= math.cos(max_incidence_angle_rad):
        return None

    slant_range_m = distance_to_plane_m / incidence_cosine
    if max_range_m is not None and slant_range_m > max_range_m:
        return None

    forward_m, right_m, down_m = mount.translation + slant_range_m * direction_frd
    return TargetPosition(
        forward_m=float(forward_m), right_m=float(right_m), down_m=float(down_m)
    )


def project_target_frd(
    position: TargetPositionLike,
    intrinsics: CameraIntrinsics,
    mount: CameraMount = NADIR_DOWN_MOUNT,
) -> ImageFrame | None:
    """Forward model: project an FRD position back into rectified image pixels.

    The inverse of :func:`locate_target_frd`, useful for testing and for predicting
    where a known target should appear.

    Returns:
        The :class:`ImageFrame` the target would land on, or ``None`` if it lies behind
        the camera.
    """
    intrinsics.validate()
    mount.validate()

    point_frd = _as_position_array(position)
    if not np.all(np.isfinite(point_frd)):
        raise ValueError(f"Point must be finite, got {point_frd.tolist()}.")

    point_cam = mount.rotation.T @ (point_frd - mount.translation)
    if point_cam[2] <= 0.0:
        return None

    return ImageFrame(
        u=intrinsics.cx + intrinsics.fx * point_cam[0] / point_cam[2],
        v=intrinsics.cy + intrinsics.fy * point_cam[1] / point_cam[2],
    )
