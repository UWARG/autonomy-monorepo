#!/usr/bin/env python3
'''
example vehicles using pyBullet
'''

import argparse
import json
import math
import os
from re import L
import socket
import struct
import sys
import time
from pathlib import Path
import pybullet as p
import cv2
import pybullet_data
import logging
import subprocess
import numpy as np
import threading
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Vector3

#drone default position and orientation
DRONE_FOV=60
DRONE_ASPECT=1.0
DRONE_NEAR=0.1
DRONE_FAR=100.0
DRONE_OFFSET_X=[1,0,0]
DRONE_OFFSET_Y=[0,1,0]
DRONE_OFFSET_Z=[0,0,1]

logging.basicConfig(level=logging.INFO)

# --- Argument parsing ---
parser = argparse.ArgumentParser(description="pybullet robot (no pyrobolearn)")
parser.add_argument("--vehicle", required=True, choices=['racecar', 'iris'], default='iris', help="vehicle type")
parser.add_argument("--fps", type=float, default=1200.0, help="physics frame rate")
parser.add_argument("--nogui", default=False, action='store_true', help="disable GUI")
args = parser.parse_args()

# --- Constants ---
RATE_HZ = args.fps
TIME_STEP = 1.0 / RATE_HZ
GRAVITY_MSS = 9.80665
CAMERA_FPS=10

# --- PyBullet initialization ---
physicsClient = p.connect(p.DIRECT if args.nogui else p.GUI)
p.setTimeStep(TIME_STEP)
p.setGravity(0, 0, -GRAVITY_MSS)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)

# --- Load environment ---
ENV_ID=p.loadURDF("plane.urdf")

object_ids = [i for i in range(p.getNumBodies())]
print(f"All Object IDs: {object_ids}")

dir_path = Path(os.path.dirname(os.path.abspath(__file__))).parent.joinpath("ardupilot/libraries/SITL/examples/JSON/pybullet/models")

# --- State ---
time_now = 0.0
last_velocity = None
vehicle = None
robot_id = None


def constrain(v, min_v, max_v):
    '''constrain a value to a range'''
    return max(min_v, min(v, max_v))


class Camera():
    def __init__(self,attached_to_object,fps=60,fov=60,aspect=1.0,near=0.1,far=100.0,height=224,width=224,camera_orientation=[0,0,1]):
        self.attached_to_object=attached_to_object
        self.fps=fps
        pos,orn=p.getBasePositionAndOrientation(self.attached_to_object)
        R=p.getMatrixFromQuaternion(orn)
        R=np.reshape(R,(3,3))
        x=R[:,0] #forward = +x
        y=R[:,1] #left = +y
        z=R[:,2] #up = +z
        
        self.view_matrix=p.computeViewMatrix(np.array(pos)+np.array(DRONE_OFFSET_X),
        np.array(x)+2*np.array(DRONE_OFFSET_X),
        np.array(z))

        self.fov=fov
        self.height=height
        self.width=width
        self.aspect=aspect
        self.near=near
        self.far=far
        self.projection_matrix=p.computeProjectionMatrixFOV(fov,width/height,near,far)
        self.rgb_img=None
        self.depth_img=None
        self.seg_img=None
    def update(self):
        time.sleep(1/CAMERA_FPS)
        pos,orn=p.getBasePositionAndOrientation(self.attached_to_object)
        R=p.getMatrixFromQuaternion(orn)
        R=np.reshape(R,(3,3))
        x=R[:,0] #forward = +x
        y=R[:,1] #left = +y
        z=R[:,2] #up = +z
        
        self.view_matrix=p.computeViewMatrix(np.array(pos)+np.array(DRONE_OFFSET_X),
        np.array(x)+2*np.array(DRONE_OFFSET_X),
        np.array(z))

        self.projection_matrix=p.computeProjectionMatrixFOV(self.fov,self.aspect,self.near,self.far)
    def capture_image(self):
        self.rgb_img,self.depth_img,self.seg_img=p.getCameraImage(self.width,self.height,self.view_matrix,self.projection_matrix,renderer=p.ER_BULLET_HARDWARE_OPENGL)[2:5]

    def camera_thread(self):
        while True:
            self.capture_image()
            self.update()
            rgba_array = np.reshape(self.rgb_img, (self.width, self.height, 4)).astype(np.uint8)
            rgb_array=cv2.cvtColor(rgba_array, cv2.COLOR_RGBA2BGR)
            ok,rgb_bytes=cv2.imencode(
                ".jpg",
                rgb_array,
                [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
            if not ok:
                logging.error("Failed to encode image")
                continue
            rgb_bytes=rgb_bytes.tobytes()
            ok,depth_bytes=cv2.imencode(
                ".jpg",
                self.depth_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
            if not ok:
                logging.error("Failed to encode depth image")
                continue
            depth_bytes=depth_bytes.tobytes()
            #if needed in future, uncomment
            """
            ok,seg_bytes=cv2.imencode(
                ".jpg",
                self.seg_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
            if not ok:
                logging.error("Failed to encode segmentation image")
                continue
            seg_bytes=seg_bytes.tobytes()
            """
            udp_header=struct.pack("QQ",len(rgb_bytes),len(depth_bytes))
            groundside_socket.sendto(udp_header+rgb_bytes+depth_bytes,('127.0.0.1', 8000))


class Range_Finder():
    def __init__(self,attached_to_object,direction=[0,0,1]):
        self.attached_to_object=attached_to_object
        pos,orn=p.getBasePositionAndOrientation(self.attached_to_object)
        self.ray_from=np.array(pos)
        self.ray_to=[
            self.ray_from[0]+direction[0],
            self.ray_from[1]+direction[1],
            self.ray_from[2]+direction[2]
        ]
        group_env=2
        for i in range(p.getNumBodies()):
            for j in range(p.getNumJoints(i)):
                p.setCollisionFilterGroupMask(j,-1,collisionFilterGroup=group_env,collisionFilterMask=1)

        
        p.setCollisionFilterGroupMask(ENV_ID,-1,collisionFilterGroup=group_env,collisionFilterMask=1) #1 is default collision group assigned to raycast
        p.rayTest(self.ray_from,self.ray_to,collisionFilterMask=group_env)
        
    def update(self):
        self.range_img=p.getRangeImage(self.attached_to_object)




class Iris(object):
    '''Iris quadcopter'''
    def __init__(self):
        global robot_id
        iris_path = os.path.join(dir_path, "iris/iris.urdf")
        robot_id = p.loadURDF(iris_path, [0, 0, 0.2])

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
                objectUniqueId=robot_id,
                linkIndex=self.motor_indices[i],
                forceObj=force,
                posObj=[0, 0, 0],
                flags=p.LINK_FRAME
                )

            # accumulate torque (about Z axis)
            total_yaw_torque += self.rotor_torque_dirs[i] * thrusts[i] * self.torque_coef

        # Apply yaw torque to body
        p.applyExternalTorque(
            objectUniqueId=robot_id,
            linkIndex=-1,
            torqueObj=[0, 0, -total_yaw_torque],
            flags=p.LINK_FRAME
        )

        # animate motor spinning
        for i in range(num_motors):
            speed = constrain(motors[i] - 1000.0, 0, 1000) * self.motor_dir[i] * self.motor_speed
            p.setJointMotorControl2(robot_id, self.motor_indices[i], p.VELOCITY_CONTROL, targetVelocity=speed)

    def reset(self):
        '''reset time and location'''
        p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.2], [0, 0, 0, 1])


class RaceCar(object):
    '''racing car'''
    def __init__(self):
        global robot_id
        robot_id = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.2])

        self.steering_joints = [4, 6]
        self.wheel_joints = [2, 3, 5, 7]
        self.steer_max = 45.0
        self.throttle_max = 200.0

        self.reset()

        print("Created RaceCar vehicle")

    def update(self, pwm):
        '''update RaceCar simulation'''
        steering = constrain((pwm[0] - 1500.0)/500.0, -1, 1) * math.radians(self.steer_max) * -1
        throttle = constrain((pwm[2] - 1500.0)/500.0, -1, 1) * self.throttle_max

        for joint in self.wheel_joints:
            p.setJointMotorControl2(robot_id, joint,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=throttle)
        for joint in self.steering_joints:
            p.setJointMotorControl2(robot_id, joint,
                                    p.POSITION_CONTROL,
                                    targetPosition=steering)

    def reset(self):
        '''reset time and location'''
        p.resetBasePositionAndOrientation(robot_id, [0, 0, 0.2], [0, 0, 0, 1])


def vector_to_AP(vec):
    return Vector3(vec[0], -vec[1], -vec[2])


def to_tuple(v3):
    return (v3.x, v3.y, v3.z)


def quaternion_to_AP(q):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([q[3], q[0], -q[1], -q[2]])


def physics_step(pwm_in,camera=None):
    global time_now, last_velocity
    vehicle.update(pwm_in)
    p.stepSimulation()
    time_now += TIME_STEP

    pos, orn = p.getBasePositionAndOrientation(robot_id)
    lin_vel, ang_vel = p.getBaseVelocity(robot_id)

    q_ap = quaternion_to_AP(orn)
    roll, pitch, yaw = q_ap.euler
    velocity = vector_to_AP(lin_vel)
    position = vector_to_AP(pos)

    dcm = q_ap.dcm
    gyro = dcm.transposed() * vector_to_AP(ang_vel)

    if last_velocity is None:
        last_velocity = velocity

    accel = (velocity - last_velocity) * (1.0 / TIME_STEP)
    last_velocity = velocity
    accel.z -= GRAVITY_MSS
    accel = dcm.transposed() * accel


    return time_now, to_tuple(gyro), to_tuple(accel), to_tuple(position), (roll, pitch, yaw), to_tuple(velocity)


# --- UDP communication setup ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))
sock.settimeout(0.1)

groundside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
groundside_socket.settimeout(0.1)

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count =1000

vehicles = {
    "iris" : Iris,
    "racecar" : RaceCar
}

if args.vehicle not in vehicles:
    print(f"Unknown vehicle {args.vehicle}")
    sys.exit(1)
vehicle = vehicles[args.vehicle]()

# show the joints
print("Vehicle joints:")
number_of_joints = p.getNumJoints(robot_id)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robot_id, joint_number)
    print(" %s : %s" % (info[0], info[1]))

# --- Main loop ---

new_camera=Camera(  
                    attached_to_object=robot_id,
                    fps=RATE_HZ,
                    fov=DRONE_FOV,
                    aspect=DRONE_ASPECT,
                    near=DRONE_NEAR,
                    far=DRONE_FAR,
                    height=224,
                    width=224)
logging.info("Created camera")
"""
while True:
    
    time.sleep(1/new_camera.fps)
    new_camera.capture_image()
    new_camera.update()
    rgba_array = np.reshape(new_camera.rgb_img, (new_camera.width, new_camera.height, 4)).astype(np.uint8)
    rgb_array=cv2.cvtColor(rgba_array, cv2.COLOR_RGBA2BGR)
    cv2.imshow("Camera",rgb_array)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        sys.exit(0)
    p.stepSimulation()
"""
def main():

    global RATE_HZ
    global TIME_STEP
    global last_SITL_frame
    global connected
    global frame_count
    global frame_time
    global print_frame_count
    global vehicle
    global robot_id
    global new_camera
    global sock

    logging.info("Starting main loop")
    if new_camera:
        thread_camera=threading.Thread(target=new_camera.camera_thread,daemon=True)
        thread_camera.start()


    #wait for the airside simulation to start
    while True:
        try:
            data, address = sock.recvfrom(100)
        except OSError:
            time.sleep(0.01)
            continue
        parse_format = 'HHI16H'
        if len(data) != struct.calcsize(parse_format):
            print(f"Bad packet size: {len(data)}")
            continue

        decoded = struct.unpack(parse_format, data)
        magic = 18458
        if decoded[0] != magic:
            print(f"Incorrect magic: {decoded[0]}")
            continue

        frame_rate_hz = decoded[1]
        frame_number = decoded[2]
        pwm = decoded[3:]
        if frame_rate_hz != RATE_HZ:
            print(f"Updated rate from {RATE_HZ} to {frame_rate_hz} Hz")
            RATE_HZ = frame_rate_hz
            TIME_STEP = 1.0 / RATE_HZ
            p.setTimeStep(TIME_STEP)

        if frame_number < last_SITL_frame:
            vehicle.reset()
            time_now = 0.0
            print("Controller reset")
        elif frame_number != last_SITL_frame + 1 and connected:
            print(f"Missed {frame_number - last_SITL_frame - 1} frames")

        last_SITL_frame = frame_number

        if not connected:
            connected = True
            print(f"Connected to {address}")

        frame_count += 1

        phys_time, gyro, accel, pos, euler, velo = physics_step(pwm,new_camera)

        json_data = {
            "timestamp": phys_time,
            "imu": {
                "gyro": gyro,
                "accel_body": accel
            },
            "position": pos,
            "attitude": euler,
            "velocity": velo
        }

        result=sock.sendto((json.dumps(json_data, separators=(',', ':')) + "\n").encode("ascii"), address)
        if result == -1:
            logging.error(f"Failed to send data to {address}")
            continue

        if frame_count % print_frame_count == 0:
            now = time.time()
            total_time = now - frame_time
            logging.info(f"{print_frame_count/total_time:.2f} fps T={phys_time:.3f} dt={total_time:.3f}")
            logging.info(f"imu: gyro={gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f}, accel={accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}, pos={pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, euler={euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}, velocity={velo[0]:.2f}, {velo[1]:.2f}, {velo[2]:.2f}")
            frame_time = now



if __name__ == "__main__":
    #start the airside simulation
    result=subprocess.run(["python", "groundside.py"])
    if result.returncode != 0:
        logging.error(f"Airside simulation failed with return code {result.returncode}")
        sys.exit(1)
    main()