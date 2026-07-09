import math

from utils.src.types import Pose, Vector3D, Quaternion, Rotation
from target_relative_drone_setpoint import target_relative_drone_setpoint 

EPS = 1e-6

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def vec_close(a: Vector3D, b: Vector3D) -> bool:
    return (
        math.isclose(a.x, b.x, abs_tol=EPS)
        and math.isclose(a.y, b.y, abs_tol=EPS)
        and math.isclose(a.z, b.z, abs_tol=EPS)
    )

def quat_close(a: Quaternion, b: Quaternion) -> bool:
    # q and -q represent the same rotation; accept either sign.
    close  = (
        math.isclose(a.w,  b.w, abs_tol=EPS)
        and math.isclose(a.x,  b.x, abs_tol=EPS)
        and math.isclose(a.y,  b.y, abs_tol=EPS)
        and math.isclose(a.z,  b.z, abs_tol=EPS)
    )
    close_ = (
        math.isclose(a.w, -b.w, abs_tol=EPS)
        and math.isclose(a.x, -b.x, abs_tol=EPS)
        and math.isclose(a.y, -b.y, abs_tol=EPS)
        and math.isclose(a.z, -b.z, abs_tol=EPS)
    )
    return close or close_

def yaw_quat(yaw: float) -> Quaternion:
    """Unit quaternion for a pure yaw rotation around Z."""
    return Quaternion(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

IDENTITY_Q = Quaternion(1.0, 0.0, 0.0, 0.0)

def identity_pose(x=0.0, y=0.0, z=0.0) -> Pose:
    return Pose(Vector3D(x, y, z), IDENTITY_Q)

def yaw_pose(yaw: float, x=0.0, y=0.0, z=0.0) -> Pose:
    return Pose(Vector3D(x, y, z), yaw_quat(yaw))


# ===========================================================================
# 1. Float yaw vs. equivalent Quaternion produce identical results
# ===========================================================================

def test_float_vs_quat_zero_yaw():
    drone = identity_pose()
    t_pos = Vector3D(5.0, 0.0, 0.0)
    rf = target_relative_drone_setpoint(t_pos, 0.0, 2.0, drone)
    rq = target_relative_drone_setpoint(t_pos, yaw_quat(0.0), 2.0, drone)
    assert vec_close(rf.position, rq.position)
    assert quat_close(rf.orientation, rq.orientation)

def test_float_vs_quat_90deg():
    drone = identity_pose()
    t_pos = Vector3D(5.0, 0.0, 0.0)
    rf = target_relative_drone_setpoint(t_pos, math.pi / 2, 2.0, drone)
    rq = target_relative_drone_setpoint(t_pos, yaw_quat(math.pi / 2), 2.0, drone)
    assert vec_close(rf.position, rq.position)
    assert quat_close(rf.orientation, rq.orientation)

def test_float_vs_quat_180deg():
    drone = identity_pose()
    t_pos = Vector3D(5.0, 0.0, 0.0)
    rf = target_relative_drone_setpoint(t_pos, math.pi, 2.0, drone)
    rq = target_relative_drone_setpoint(t_pos, yaw_quat(math.pi), 2.0, drone)
    assert vec_close(rf.position, rq.position)
    assert quat_close(rf.orientation, rq.orientation)

def test_float_vs_quat_neg90deg():
    drone = identity_pose()
    t_pos = Vector3D(5.0, 0.0, 0.0)
    rf = target_relative_drone_setpoint(t_pos, -math.pi / 2, 2.0, drone)
    rq = target_relative_drone_setpoint(t_pos, yaw_quat(-math.pi / 2), 2.0, drone)
    assert vec_close(rf.position, rq.position)
    assert quat_close(rf.orientation, rq.orientation)

def test_float_vs_quat_arbitrary_yaw():
    drone = identity_pose()
    t_pos = Vector3D(5.0, 0.0, 0.0)
    yaw = 1.23
    rf = target_relative_drone_setpoint(t_pos, yaw, 2.0, drone)
    rq = target_relative_drone_setpoint(t_pos, yaw_quat(yaw), 2.0, drone)
    assert vec_close(rf.position, rq.position)
    assert quat_close(rf.orientation, rq.orientation)


# ===========================================================================
# 2. Setpoint position — drone at origin + identity so local == world
# ===========================================================================

def test_position_zero_yaw_positive_distance():
    # forward = (1,0,0); setpoint = (5,0,0) + 2*(1,0,0) = (7,0,0)
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), 0.0, 2.0, identity_pose())
    assert vec_close(result.position, Vector3D(7.0, 0.0, 0.0))

def test_position_zero_yaw_zero_distance():
    # setpoint == target position
    result = target_relative_drone_setpoint(Vector3D(3.0, 1.0, 2.0), 0.0, 0.0, identity_pose())
    assert vec_close(result.position, Vector3D(3.0, 1.0, 2.0))

def test_position_90deg_yaw():
    # forward = (0,1,0); setpoint = (0,0,0) + 2*(0,1,0) = (0,2,0)
    result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), math.pi / 2, 2.0, identity_pose())
    assert vec_close(result.position, Vector3D(0.0, 2.0, 0.0))

def test_position_180deg_yaw():
    # forward = (-1,0,0); setpoint = (5,0,0) + 2*(-1,0,0) = (3,0,0)
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), math.pi, 2.0, identity_pose())
    assert vec_close(result.position, Vector3D(3.0, 0.0, 0.0))

def test_position_neg90deg_yaw():
    # forward = (0,-1,0); setpoint = (0,0,0) + 2*(0,-1,0) = (0,-2,0)
    result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), -math.pi / 2, 2.0, identity_pose())
    assert vec_close(result.position, Vector3D(0.0, -2.0, 0.0))

def test_position_arbitrary_yaw():
    yaw = 0.7
    dist = 3.0
    t_pos = Vector3D(1.0, 2.0, 0.0)
    result = target_relative_drone_setpoint(t_pos, yaw, dist, identity_pose())
    expected = Vector3D(t_pos.x + dist * math.cos(yaw), t_pos.y + dist * math.sin(yaw), 0.0)
    assert vec_close(result.position, expected)

def test_position_translated_target():
    result = target_relative_drone_setpoint(Vector3D(100.0, 50.0, -10.0), 0.0, 5.0, identity_pose())
    assert vec_close(result.position, Vector3D(105.0, 50.0, -10.0))

def test_position_small_distance():
    dist = 1e-9
    result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), 0.0, dist, identity_pose())
    assert vec_close(result.position, Vector3D(dist, 0.0, 0.0))

def test_position_distance_scales_linearly():
    # position offset should grow linearly — spot-check three distances
    for dist in [0.5, 5.0, 100.0]:
        result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), 0.0, dist, identity_pose())
        assert vec_close(result.position, Vector3D(dist, 0.0, 0.0)), f"failed for dist={dist}"


# ===========================================================================
# 3. Setpoint orientation — drone should face the target
# ===========================================================================

def rotate_by_quat(q: Quaternion, v: Vector3D) -> Vector3D:
    """Rotate vector v by quaternion q via sandwich product q*v*q'."""
    return (q * v.to_pure_quaternion() * q.c()).to_vector3d()

def test_orientation_zero_yaw_drone_faces_target():
    # Target at (5,0,0), yaw=0 → forward=(1,0,0), setpoint at (7,0,0).
    # Drone forward rotated by returned orientation should point (7→5) = (-1,0,0).
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), 0.0, 2.0, identity_pose())
    rotated_forward = rotate_by_quat(result.orientation, Vector3D(1.0, 0.0, 0.0))
    assert vec_close(rotated_forward, Vector3D(-1.0, 0.0, 0.0))

def test_orientation_90deg_yaw_drone_faces_target():
    # Target at origin, yaw=90° → forward=(0,1,0), setpoint at (0,2,0).
    # Drone forward rotated by returned orientation should point (0,2→0) = (0,-1,0).
    result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), math.pi / 2, 2.0, identity_pose())
    rotated_forward = rotate_by_quat(result.orientation, Vector3D(1.0, 0.0, 0.0))
    assert vec_close(rotated_forward, Vector3D(0.0, -1.0, 0.0))

def test_orientation_180deg_yaw_drone_faces_target():
    # Target at (5,0,0), yaw=180° → forward=(-1,0,0), setpoint at (3,0,0).
    # Drone forward rotated by returned orientation should point (3→5) = (1,0,0).
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), math.pi, 2.0, identity_pose())
    rotated_forward = rotate_by_quat(result.orientation, Vector3D(1.0, 0.0, 0.0))
    assert vec_close(rotated_forward, Vector3D(1.0, 0.0, 0.0))


# ===========================================================================
# 4. Frame conversion — drone at non-identity pose
# ===========================================================================

def test_frame_drone_yaw_90_position():
    # Drone at origin, facing left (90° yaw).
    # World setpoint = (7,0,0).
    # In drone local frame (X=world Y, Y=-world X): local = (0,-7,0).
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), 0.0, 2.0, yaw_pose(math.pi / 2))
    assert vec_close(result.position, Vector3D(0.0, -7.0, 0.0))

def test_frame_drone_yaw_180_position():
    # Drone facing backward. World setpoint = (7,0,0) → local = (-7,0,0).
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), 0.0, 2.0, yaw_pose(math.pi))
    assert vec_close(result.position, Vector3D(-7.0, 0.0, 0.0))

def test_frame_translated_drone():
    # Drone at (3,0,0) identity. World setpoint = (7,0,0). Local = (4,0,0).
    result = target_relative_drone_setpoint(Vector3D(5.0, 0.0, 0.0), 0.0, 2.0, identity_pose(3.0, 0.0, 0.0))
    assert vec_close(result.position, Vector3D(4.0, 0.0, 0.0))

def test_frame_translated_drone_and_target():
    # Drone at (1,1,0). Target at (4,1,0), yaw=0, dist=1 → world setpoint=(5,1,0).
    # Local = (4,0,0).
    result = target_relative_drone_setpoint(Vector3D(4.0, 1.0, 0.0), 0.0, 1.0, identity_pose(1.0, 1.0, 0.0))
    assert vec_close(result.position, Vector3D(4.0, 0.0, 0.0))

def test_frame_rotation_preserves_magnitude():
    # Rotating the drone frame must not change the distance to the setpoint.
    t_pos = Vector3D(4.0, 0.0, 0.0)
    for drone_yaw in [0.0, math.pi / 2, math.pi, -math.pi / 2]:
        result = target_relative_drone_setpoint(t_pos, 0.0, 1.0, yaw_pose(drone_yaw))
        # World setpoint is at (5,0,0), drone at origin → magnitude must be 5.
        mag = math.sqrt(result.position.x**2 + result.position.y**2 + result.position.z**2)
        assert math.isclose(mag, 5.0, abs_tol=EPS), f"failed for drone_yaw={drone_yaw}"

def test_frame_arbitrary_drone_yaw_and_position():
    # Sanity check: magnitude is preserved through an arbitrary frame change.
    drone_pos = Vector3D(2.0, 3.0, 0.0)
    drone = Pose(drone_pos, yaw_quat(0.4))
    t_pos = Vector3D(8.0, 1.0, 0.0)
    yaw_t = 1.1
    dist = 1.5
    result = target_relative_drone_setpoint(t_pos, yaw_t, dist, drone)
    sp_world = Vector3D(
        t_pos.x + dist * math.cos(yaw_t),
        t_pos.y + dist * math.sin(yaw_t),
        0.0,
    )
    diff = Vector3D(sp_world.x - drone_pos.x, sp_world.y - drone_pos.y, sp_world.z - drone_pos.z)
    expected_mag = math.sqrt(diff.x**2 + diff.y**2 + diff.z**2)
    actual_mag = math.sqrt(result.position.x**2 + result.position.y**2 + result.position.z**2)
    assert math.isclose(actual_mag, expected_mag, abs_tol=EPS)


# ===========================================================================
# 5. Singular target directions (forward collinear with world X)
# ===========================================================================

def test_singular_forward_exactly_world_x():
    # yaw=0 → forward=(1,0,0). Must not crash and must place setpoint correctly.
    result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), 0.0, 3.0, identity_pose())
    assert vec_close(result.position, Vector3D(3.0, 0.0, 0.0))

def test_singular_forward_exactly_negative_world_x():
    # yaw=π → forward=(-1,0,0). Must not crash.
    result = target_relative_drone_setpoint(Vector3D(0.0, 0.0, 0.0), math.pi, 3.0, identity_pose())
    assert vec_close(result.position, Vector3D(-3.0, 0.0, 0.0))


# ===========================================================================
# 6. Coincident positions (drone and target at the same point)
# ===========================================================================

def test_coincident_zero_distance():
    # Setpoint == target position == drone position; local position should be zero.
    drone = identity_pose(2.0, 2.0, 0.0)
    result = target_relative_drone_setpoint(Vector3D(2.0, 2.0, 0.0), 0.0, 0.0, drone)
    assert vec_close(result.position, Vector3D(0.0, 0.0, 0.0))

def test_coincident_nonzero_distance():
    # Drone at (5,5,0), target at (5,5,0), yaw=0, dist=4 → world setpoint=(9,5,0).
    # Local = (4,0,0).
    drone = identity_pose(5.0, 5.0, 0.0)
    result = target_relative_drone_setpoint(Vector3D(5.0, 5.0, 0.0), 0.0, 4.0, drone)
    assert vec_close(result.position, Vector3D(4.0, 0.0, 0.0))