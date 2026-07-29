"""Iris quadcopter PyBullet model."""

import logging
import os

import pybullet as p

import state


def constrain(value, min_value, max_value):
    """Constrain a value to a range."""
    return max(min_value, min(value, max_value))


class Iris:
    """Iris quadcopter"""

    def __init__(self):
        state.robot_id = p.loadURDF("hoop.urdf", [0, 0, 0.2])

        self.motor_indices = [1, 2, 3, 4]
        self.motor_dir = [1, 1, -1, -1]
        self.motor_speed = 5
        self.thrust_scale = 0.01

        self.rotor_torque_dirs = [1, 1, -1, -1]
        self.torque_coef = 0.001

        arm_length = 0.2
        self.rotor_positions = [
            [arm_length, -arm_length, 0],
            [-arm_length, arm_length, 0],
            [arm_length, arm_length, 0],
            [-arm_length, arm_length, 0],
        ]
        self.reset()
        logging.info("Created Iris vehicle")

    def update(self, pwm):
        """Update Iris simulation."""
        num_motors = 4
        motors = pwm[:num_motors]

        thrusts = [constrain(p - 1000, 0, 1000) * self.thrust_scale for p in motors]

        total_yaw_torque = 0.0

        for i in range(num_motors):
            force = [0, 0, thrusts[i]]
            p.applyExternalForce(
                objectUniqueId=state.robot_id,
                linkIndex=self.motor_indices[i],
                forceObj=force,
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME,
            )

            total_yaw_torque += (
                self.rotor_torque_dirs[i] * thrusts[i] * self.torque_coef
            )

        p.applyExternalTorque(
            objectUniqueId=state.robot_id,
            linkIndex=-1,
            torqueObj=[0, 0, -total_yaw_torque],
            flags=p.LINK_FRAME,
        )

        for i in range(num_motors):
            speed = (
                constrain(motors[i] - 1000.0, 0, 1000)
                * self.motor_dir[i]
                * self.motor_speed
            )
            p.setJointMotorControl2(
                state.robot_id,
                self.motor_indices[i],
                p.VELOCITY_CONTROL,
                targetVelocity=speed,
            )

    def reset(self):
        """Reset time and location."""
        p.resetBasePositionAndOrientation(state.robot_id, [0, 0, 0.2], [0, 0, 0, 1])
