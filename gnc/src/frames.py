import math

from utils import Rotation, Vector3D

MM_PER_M = 1000.0

def camera_to_body(
    cam_x_mm: float,
    cam_y_mm: float,
    cam_z_mm: float,
    mount_pitch_rad: float = 0.0,
    mount_roll_rad: float = 0.0,
) -> Vector3D:
    """Camera-optical XYZ (mm) -> body FRD vector (m), corrected for mount tilt.
    cam_x = right, cam_y = down, cam_z = forward (depth). Returns Vector3D with
    (x=forward, y=right, z=down) in metres. ``mount_pitch_rad`` > 0 means the
    camera is tilted nose-down by that angle; it is rotated back to level.
    """
    forward = cam_z_mm / MM_PER_M
    right = cam_x_mm / MM_PER_M
    down = cam_y_mm / MM_PER_M
    vec = Vector3D(forward, right, down)

    # Remove the mount pitch about the body right axis (+y), then roll about the
    # body forward axis (+x). A nose-down pitch of +theta is undone by rotating
    # the measured vector by -theta about the right axis.
    if mount_pitch_rad:
        vec = Rotation(Vector3D(0.0, 1.0, 0.0), -mount_pitch_rad).rotate_vector3d(vec)
    if mount_roll_rad:
        vec = Rotation(Vector3D(1.0, 0.0, 0.0), -mount_roll_rad).rotate_vector3d(vec)
    return vec


def body_to_ned(body: Vector3D, yaw_rad: float) -> Vector3D:
    """Rotate a body FRD vector into world NED using the drone heading"""
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    north = body.x * cos_y - body.y * sin_y
    east = body.x * sin_y + body.y * cos_y
    down = body.z
    return Vector3D(north, east, down)


def ned_to_enu(ned: Vector3D) -> Vector3D:
    """Convert a world NED vector to ENU (north,east,down) -> (east,north,up)."""
    return Vector3D(ned.y, ned.x, -ned.z)


def yaw_to_face(body: Vector3D) -> float:
    """Yaw error (rad) needed to face a body-frame target; +ve = target to the right."""
    return math.atan2(body.y, body.x)
