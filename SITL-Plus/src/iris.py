import logging
import os

import pybullet as p

import state


def constrain(v, min_v, max_v):
    '''constrain a value to a range'''
    return max(min_v, min(v, max_v))


class Iris(object):
    '''Iris quadcopter'''
    def __init__(self):
        iris_path = os.path.join(state.dir_path, "iris/iris.urdf")
        state.robot_id = p.loadURDF(iris_path, [0, 0, 0.2])

        self.motor_indices = [1, 2, 3, 4]
        self.motor_dir = [1, 1, -1, -1]
        self.motor_speed = 5 # visual speed
        self.thrust_scale = 0.01

        # positive for CCW, negative for CW (quad-X layout)
        self.rotor_torque_dirs = [1, 1, -1, -1]
        self.torque_coef = 0.001  # Nm per unit thrust (tunable)

        # physical layout
        L = 0.2  # arm length
        self.rotor_positions = [
            [L, -L, 0],   # motor 1, Front-Right
            [-L, L, 0],   # motor 2, Rear-Left
            [L, L, 0],   # motor 3, Front-Left
            [-L, L, 0],   # motor 4, Rear-Right
        ]
        self.reset()
        logging.info("Created Iris vehicle")

    def update(self, pwm):
        '''update Iris simulation'''
        num_motors = 4
        motors = pwm[:num_motors]

        # scale PWM to thrust (N) and torque (Nm)
        thrusts = [constrain(p - 1000, 0, 1000) * self.thrust_scale for p in motors]

        total_yaw_torque = 0.0

        for i in range(num_motors):
            force = [0, 0, thrusts[i]]
            p.applyExternalForce(
                objectUniqueId=state.robot_id,
                linkIndex=self.motor_indices[i],
                forceObj=force,
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME
                )

            # accumulate torque (about Z axis)
            total_yaw_torque += self.rotor_torque_dirs[i] * thrusts[i] * self.torque_coef

        # Apply yaw torque to body
        p.applyExternalTorque(
            objectUniqueId=state.robot_id,
            linkIndex=-1,
            torqueObj=[0, 0, -total_yaw_torque],
            flags=p.LINK_FRAME
        )

        # animate motor spinning
        for i in range(num_motors):
            speed = constrain(motors[i] - 1000.0, 0, 1000) * self.motor_dir[i] * self.motor_speed
            p.setJointMotorControl2(state.robot_id, self.motor_indices[i], p.VELOCITY_CONTROL, targetVelocity=speed)

    def reset(self):
        '''reset time and location'''
        p.resetBasePositionAndOrientation(state.robot_id, [0, 0, 0.2], [0, 0, 0, 1])
