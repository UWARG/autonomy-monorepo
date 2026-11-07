"""Tests for scene Object loading."""

import math

import pybullet as p

from object import Object


def test_object_init(_bullet_connect):
    """Primitive and URDF objects load with expected poses."""
    sphere = Object(name="sphere", position=[0, 0, 0], orientation=[0, 0, 0], radius=1)
    barrel = Object(
        name="barrel", position=[0, 4, 0], orientation=[0, 0, 2], radius=0.5, height=1
    )
    hoop = Object(
        name="hoop", position=[1, 2, 3], orientation=[math.pi / 2, 0, 0], radius=1
    )
    r2d2 = Object(
        name="r2d2.urdf",
        position=[4, 5, 6],
        orientation=[0, math.pi / 2, math.pi / 2],
        scale=1,
    )

    sphere.initialize()
    barrel.initialize()
    hoop.initialize()
    r2d2.initialize()

    assert sphere.id is not None
    assert barrel.id is not None
    assert hoop.id is not None
    assert r2d2.id is not None

    pos, orn = p.getBasePositionAndOrientation(sphere.id)
    assert (
        math.isclose(pos[0], 0, rel_tol=1e-3)
        and math.isclose(pos[1], 0, rel_tol=1e-3)
        and math.isclose(pos[2], 0, rel_tol=1e-3)
    )
    assert (
        math.isclose(orn[0], 0, rel_tol=1e-3)
        and math.isclose(orn[1], 0, rel_tol=1e-3)
        and math.isclose(orn[2], 0, rel_tol=1e-3)
        and math.isclose(orn[3], 1, rel_tol=1e-3)
    )
    pos, orn = p.getBasePositionAndOrientation(barrel.id)
    assert (
        math.isclose(pos[0], 0, rel_tol=1e-3)
        and math.isclose(pos[1], 4, rel_tol=1e-3)
        and math.isclose(pos[2], 0, rel_tol=1e-3)
    )
    euler = p.getEulerFromQuaternion(orn)
    assert (
        math.isclose(euler[0], 0, rel_tol=1e-3)
        and math.isclose(euler[1], 0, rel_tol=1e-3)
        and math.isclose(euler[2], 2, rel_tol=1e-3)
    )
    pos, orn = p.getBasePositionAndOrientation(hoop.id)
    assert (
        math.isclose(pos[0], 1, rel_tol=1e-3)
        and math.isclose(pos[1], 2, rel_tol=1e-3)
        and math.isclose(pos[2], 3, rel_tol=1e-3)
    )
    euler = p.getEulerFromQuaternion(orn)
    assert (
        math.isclose(euler[0], math.pi / 2, rel_tol=1e-3)
        and math.isclose(euler[1], 0, rel_tol=1e-3)
        and math.isclose(euler[2], 0, rel_tol=1e-3)
    )
    pos, orn = p.getBasePositionAndOrientation(r2d2.id)
    assert (
        math.isclose(pos[0], 4, rel_tol=1e-3)
        and math.isclose(pos[1], 5, rel_tol=1e-3)
        and math.isclose(pos[2], 6, rel_tol=1e-3)
    )
    euler = list(p.getEulerFromQuaternion(orn))
    for index, angle in enumerate(euler):
        if angle > math.pi:
            euler[index] = angle - 2 * math.pi
        if angle < -math.pi:
            euler[index] = angle + 2 * math.pi
    assert (
        math.isclose(euler[0], 0, rel_tol=1e-3)
        and math.isclose(euler[1], math.pi / 2, rel_tol=1e-3)
        and math.isclose(euler[2], math.pi / 2, rel_tol=1e-3)
    )
