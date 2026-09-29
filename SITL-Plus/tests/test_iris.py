"""Tests for the Iris vehicle model."""

import json
from unittest.mock import Mock

import pybullet as p
import pytest

from drone import Drone, SERVO_PACKET
from range_finder import Range_Finder

import iris


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
    pos, orient = p.getBasePositionAndOrientation(iris_obj.robot_id)
    assert pos == (0, 0, 0.2)
    assert orient == (0, 0, 0, 1)


def test_two_drones_share_one_step(_world, monkeypatch):
    """Separate controllers drive separate bodies on the same physics clock."""
    first = Drone("first", "sitl-1", iris.Iris((10, 0, 2)), "localhost")
    second = Drone("second", "sitl-2", iris.Iris((13, 0, 4)), "localhost", 10)
    sock = Mock()

    def packet(frame, pwm):
        return SERVO_PACKET.pack(18458, 800, frame, *([pwm] * 16))

    first.receive(packet(0, 1100), ("10.0.0.2", 9002), sock)
    second.receive(packet(0, 1500), ("10.0.0.3", 9002), sock)
    step = Mock(wraps=p.stepSimulation)
    monkeypatch.setattr(p, "stepSimulation", step)
    before = _world.time_now
    _world.step([first, second])
    step.assert_called_once()
    assert _world.time_now == pytest.approx(before + _world.time_step)
    assert first.vehicle.robot_id != second.vehicle.robot_id
    assert first.pwm[0] == 1100
    assert second.pwm[0] == 1500
    for drone in (first, second):
        drone.send(sock, _world.time_now, _world.time_step)
        telemetry = json.loads(drone.reply)
        assert telemetry["timestamp"] == _world.time_now
        pos, _ = p.getBasePositionAndOrientation(drone.vehicle.robot_id)
        assert telemetry["position"] == pytest.approx([pos[0], -pos[1], -pos[2]])
        assert not drone.pending
    assert sock.sendto.call_args_list[0].args[1] == first.address
    assert sock.sendto.call_args_list[2].args[1] == second.address
    # A retransmitted frame gets the same response without another physics step.
    first.receive(packet(0, 1100), first.address, sock)
    assert not first.pending
    assert sock.sendto.call_args.args == (first.reply, first.address)
    second_pos = p.getBasePositionAndOrientation(second.vehicle.robot_id)
    first.receive(packet(1, 1200), first.address, sock)
    first.receive(packet(0, 1000), first.address, sock)
    assert p.getBasePositionAndOrientation(first.vehicle.robot_id)[0] == (10, 0, 2)
    assert p.getBasePositionAndOrientation(second.vehicle.robot_id) == second_pos
    sensor = Range_Finder(6004, attached_to_object=second.vehicle.robot_id)
    sensor.update()
    assert sensor.range == pytest.approx(second_pos[0][2], abs=0.1)
    for drone in (first, second):
        p.removeBody(drone.vehicle.robot_id)
