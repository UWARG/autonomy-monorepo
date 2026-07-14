"""Gate 0 -- frame and unit conversions, every sign pinned."""

import math

from frames import body_to_ned, camera_to_body, ned_to_enu, yaw_to_face
from utils import Rotation, Vector3D

TOL = 1e-9


def _close(v: Vector3D, fwd: float, right: float, down: float, tol: float = 1e-6) -> bool:
    return abs(v.x - fwd) < tol and abs(v.y - right) < tol and abs(v.z - down) < tol


def test_mm_to_m_and_axis_remap_no_tilt():
    # cam z=forward, x=right, y=down (mm) -> body (forward, right, down) in metres
    assert _close(camera_to_body(0, 0, 1000), 1.0, 0.0, 0.0)  # 1 m ahead
    assert _close(camera_to_body(1000, 0, 0), 0.0, 1.0, 0.0)  # 1 m to the right
    assert _close(camera_to_body(0, 1000, 0), 0.0, 0.0, 1.0)  # 1 m below


def test_camera_pitch_tilts_optical_axis_down():
    # A target along a nose-down optical axis maps to forward+down in the body frame.
    theta = 0.3
    v = camera_to_body(0, 0, 1000, mount_pitch_rad=theta)
    assert abs(v.x - math.cos(theta)) < 1e-6  # forward
    assert abs(v.y) < 1e-6  # right
    assert abs(v.z - math.sin(theta)) < 1e-6  # down (positive = below)


def test_camera_pitch_correction_is_exact_inverse():
    # Forward-model a true body vector into a tilted camera reading, then confirm
    # camera_to_body recovers it exactly (catches any sign/axis slip).
    theta = 0.4
    for true_fwd, true_right, true_down in [(2.0, 0.5, -0.3), (1.0, -1.0, 0.8)]:
        true_body = Vector3D(true_fwd, true_right, true_down)
        aligned = Rotation(Vector3D(0, 1, 0), theta).rotate_vector3d(true_body)
        cam_z_mm, cam_x_mm, cam_y_mm = aligned.x * 1000, aligned.y * 1000, aligned.z * 1000
        recovered = camera_to_body(cam_x_mm, cam_y_mm, cam_z_mm, mount_pitch_rad=theta)
        assert _close(recovered, true_fwd, true_right, true_down)


def test_body_to_ned_yaw_zero_is_identity():
    n = body_to_ned(Vector3D(1.0, 2.0, 3.0), 0.0)
    assert _close(n, 1.0, 2.0, 3.0)  # north=fwd, east=right, down=down


def test_body_to_ned_yaw_ninety_rotates_forward_to_east():
    n = body_to_ned(Vector3D(1.0, 0.0, 0.0), math.pi / 2)
    assert abs(n.x) < 1e-9  # north ~ 0
    assert abs(n.y - 1.0) < 1e-9  # east ~ 1 (facing east, forward -> east)
    assert abs(n.z) < 1e-9


def test_ned_to_enu_swaps_and_flips_down():
    e = ned_to_enu(Vector3D(1.0, 2.0, 3.0))  # (north, east, down)
    assert _close(e, 2.0, 1.0, -3.0)  # (east, north, up)


def test_yaw_to_face_sign():
    assert abs(yaw_to_face(Vector3D(1.0, 1.0, 0.0)) - math.pi / 4) < 1e-9  # target right -> +ve
    assert abs(yaw_to_face(Vector3D(1.0, -1.0, 0.0)) + math.pi / 4) < 1e-9  # target left -> -ve
