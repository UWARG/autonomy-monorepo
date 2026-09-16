"""Tests for the Iris vehicle model."""

import pybullet as p

import iris
import state


def test_iris_init(_bullet_connect):
    """Iris initialization sets motor geometry and parameters."""
    iris_obj = iris.Iris()
    assert iris_obj.motor_indices == [1, 2, 3, 4]
    assert iris_obj.motor_dir == [1, 1, -1, -1]
    assert iris_obj.motor_speed == 5
    assert iris_obj.thrust_scale == 0.01
    assert iris_obj.rotor_torque_dirs == [1, 1, -1, -1]
    assert iris_obj.torque_coef == 0.001
    arm_length = 0.2
    assert iris_obj.rotor_positions == [
        [arm_length, -arm_length, 0],
        [-arm_length, arm_length, 0],
        [arm_length, arm_length, 0],
        [-arm_length, arm_length, 0],
    ]


def test_iris_reset(_bullet_connect, iris_obj):
    """Resetting Iris restores the default pose."""
    iris_obj.reset()
    pos, orient = p.getBasePositionAndOrientation(state.robot_id)
    assert pos == (0, 0, 0.2)
    assert orient == (0, 0, 0, 1)
