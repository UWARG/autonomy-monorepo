"""Per-drone vehicle, ArduPilot connection, and telemetry state."""

import json
import logging
import struct

import pybullet as p
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Vector3

from environment import GRAVITY_MSS

SERVO_PACKET = struct.Struct("<HHI16H")


def vector_to_AP(vec):  # pylint: disable=invalid-name
    """Convert a PyBullet vector to ArduPilot coordinates."""
    return Vector3(vec[0], -vec[1], -vec[2])


def to_tuple(vector):
    """Convert a Vector3 to a tuple."""
    return (vector.x, vector.y, vector.z)


def quaternion_to_AP(quaternion):  # pylint: disable=invalid-name
    """Convert a PyBullet quaternion to ArduPilot coordinates."""
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])


class Drone:
    """Keep each controller's frames and physics state independent."""

    def __init__(self, name, host, vehicle, sensor_host, port_offset=0):
        self.name = name
        self.host = host
        self.vehicle = vehicle
        self.sensor_host = sensor_host
        self.port_offset = port_offset
        self.address = None
        self.last_frame = -1
        self.last_velocity = None
        self.pwm = None
        self.reply = None
        self.pending = False

    def receive(self, data, address, sock):
        """Queue controls; resend cached telemetry for duplicate frames."""
        if len(data) != SERVO_PACKET.size:
            return
        magic, _rate, frame, *pwm = SERVO_PACKET.unpack(data)
        if magic != 18458:
            return
        if frame == self.last_frame:
            if self.reply is not None:
                sock.sendto(self.reply, address)
            return
        if frame < self.last_frame:
            # A restarted SITL begins at frame zero. Ignore stale UDP packets.
            if frame != 0:
                return
            self.vehicle.reset()
            self.last_velocity = None
            logging.info("Controller reset: %s", self.name)
        self.address = address
        self.last_frame = frame
        self.pwm = pwm
        self.reply = None
        self.pending = True

    def send(self, sock, timestamp, time_step):
        """Return this body's telemetry to its controller and sensor consumer."""
        telemetry = self.telemetry(timestamp, time_step)
        self.reply = (json.dumps(telemetry, separators=(",", ":")) + "\n").encode(
            "ascii"
        )
        sock.sendto(self.reply, self.address)
        sock.sendto(
            struct.pack("ffffff", *telemetry["position"], *telemetry["attitude"]),
            (self.sensor_host, 4000 + self.port_offset),
        )
        self.pending = False

    def telemetry(self, timestamp, time_step):
        """Read this drone after the shared physics step."""
        pos, orn = p.getBasePositionAndOrientation(self.vehicle.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.vehicle.robot_id)

        q_ap = quaternion_to_AP(orn)
        roll, pitch, yaw = q_ap.euler
        velocity = vector_to_AP(lin_vel)
        position = vector_to_AP(pos)

        dcm = q_ap.dcm
        gyro = dcm.transposed() * vector_to_AP(ang_vel)

        if self.last_velocity is None:
            self.last_velocity = velocity

        accel = (velocity - self.last_velocity) * (1.0 / time_step)
        self.last_velocity = velocity
        accel.z -= GRAVITY_MSS
        accel = dcm.transposed() * accel

        return {
            "timestamp": timestamp,
            "imu": {"gyro": to_tuple(gyro), "accel_body": to_tuple(accel)},
            "position": to_tuple(position),
            "attitude": (roll, pitch, yaw),
            "velocity": to_tuple(velocity),
        }
