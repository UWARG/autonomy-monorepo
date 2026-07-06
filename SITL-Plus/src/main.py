#!/usr/bin/env python3
'''
example vehicles using pyBullet
'''

import argparse
import json
import math
import os
import socket
import struct
import time
from pathlib import Path
import sensor_ports
import pybullet as p
import pybullet_data
import logging
import threading
from scipy.spatial.transform import Rotation as R
import rerun as rr
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Vector3


from camera import Camera
from range_finder import Range_Finder
from object import Object
from iris import Iris
import state
import constants

logging.basicConfig(level=logging.INFO)

# --- Argument parsing ---
parser = argparse.ArgumentParser(description="pybullet robot (no pyrobolearn)")
parser.add_argument("--nogui", default=False, action='store_true', help="disable GUI")
args = parser.parse_args()

# --- Constants ---
RATE_HZ = 800
TIME_STEP = 1.0 / RATE_HZ
GRAVITY_MSS = 9.80665
MISSED_FRAMES_ALLOWED=5
TELEM_PORT=4000
HOST=os.getenv("SENSOR_HOST")
if HOST is None:
    raise ValueError("SENSOR_HOST is not set")

# --- PyBullet initialization ---
physicsClient = p.connect(p.DIRECT if args.nogui else p.GUI)
p.setTimeStep(TIME_STEP)
p.setGravity(0, 0, -GRAVITY_MSS)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
if len(sensor_ports.CAMERA_PORTS)<2:
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
else:
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

# --- Load environment ---
ENV_ID=p.loadURDF("plane.urdf")

object_ids = [i for i in range(p.getNumBodies())]
print(f"All Object IDs: {object_ids}")

state.dir_path = Path(os.path.dirname(os.path.abspath(__file__))).joinpath("ardupilot/libraries/SITL/examples/JSON/pybullet/models")

# --- State ---
time_now = 0.0
last_velocity = None
vehicle = None

#rerun
rr.init("sitl-plus")
rr.connect_grpc("rerun+http://host.docker.internal:9876/proxy")
cameras=[]
range_finders=[]

def vector_to_AP(vec):
    return Vector3(vec[0], -vec[1], -vec[2])


def to_tuple(v3):
    return (v3.x, v3.y, v3.z)


def quaternion_to_AP(q):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([q[3], q[0], -q[1], -q[2]])


def physics_step(pwm_in):
    global time_now, last_velocity
    vehicle.update(pwm_in)
    p.stepSimulation()
    time_now += TIME_STEP

    pos, orn = p.getBasePositionAndOrientation(state.robot_id)
    lin_vel, ang_vel = p.getBaseVelocity(state.robot_id)

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


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9002))
sock.settimeout(0.5) #0.5 seconds is the maximum time to wait for a packet so frame does not get missed

state.airside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state.airside_socket.settimeout(0.1)

last_SITL_frame = -1
connected = False
frame_count = 0
frame_time = time.time()
print_frame_count =1000

vehicle = Iris()

# show the joints
print("Vehicle joints:")
number_of_joints = p.getNumJoints(state.robot_id)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(state.robot_id, joint_number)
    print(" %s : %s" % (info[0], info[1]))

# --- Main loop ---
for key,value in sensor_ports.CAMERA_PORTS.items():
    cameras.append(Camera(  
                    attached_to_object=state.robot_id,
                    port=key,
                    direction=value["direction"],
                    fov=value["fov"],
                    near=value["near"],
                    far=value["far"],
                    height=value["height"],
                    width=value["width"]
                    ))
    
for key,value in sensor_ports.RANGE_FINDER_PORTS.items():
    range_finders.append(Range_Finder(
                    port=key,
                    direction=value["direction"],
                    dist=value["dist"]
                    ))

logging.info("Created camera")
logging.info("Created range finder")

def update_camera_range_finder():
    while True:
        time.sleep(1/constants.CAMERA_FPS)
        state.update+=1


def main():
    rr.log("drone",rr.Boxes3D(centers=[[0,0,0]], half_sizes=[[1,0.5,0.2]],colors=[[255,0,0]],fill_mode="solid"))
    update_thread=threading.Thread(target=update_camera_range_finder,daemon=True)
    update_thread.start()

    global RATE_HZ
    global TIME_STEP
    global last_SITL_frame
    global connected
    global frame_count
    global frame_time
    global print_frame_count
    global vehicle
    global sock

    logging.info("Starting main loop")
    for camera in cameras:
        thread_camera=threading.Thread(target=camera.camera_thread,daemon=True)
        thread_camera.start()
    for range_finder in range_finders:
        thread_range_finder=threading.Thread(target=range_finder.range_thread,daemon=True)
        thread_range_finder.start()

    #define some objects
    objects=[
        Object("r2d2.urdf",position=[4,6,0],orientation=[0,0,math.pi/2]),
        Object("sphere_small.urdf",position=[2,2,0],orientation=[math.pi/2,0,0],scale=5),
        Object("barrel",position=[2,2,3],orientation=[0,0,0],scale=1,radius=0.5,height=1),
        Object("sphere",position=[1,1,3],orientation=[0,0,0],scale=1,radius=0.5),
        Object("hoop",position=[-3,1,3],orientation=[math.pi/2,0,0],scale=1,radius=1),
    ]
    for obj in objects:
        obj.initialize()


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


        #fast reset
        keys=p.getKeyboardEvents()
        for key_code, event_state in keys.items():
            if key_code == ord('q') and event_state & p.KEY_WAS_TRIGGERED:
                frame_number=0

        if frame_number < last_SITL_frame-MISSED_FRAMES_ALLOWED: #to protect against out of order frames (usually skips at most 4 on 800 fps, adjust based on fps)
            print(f"frame_number: {frame_number} last_SITL_frame: {last_SITL_frame}")
            vehicle.reset()
            print("Controller reset")
        elif frame_number != last_SITL_frame + 1 and connected:
            print(f"Missed {frame_number - last_SITL_frame - 1} frames")

        last_SITL_frame = frame_number

        if not connected:
            connected = True
            print(f"Connected to {address}")

        frame_count += 1

        phys_time, gyro, accel, pos, euler, velo = physics_step(pwm)

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
        #since we have different coordiante systems for ardupilot and rerun, convert back to rerun format
        new_position=[pos[0], -pos[1], -pos[2]]
        quaternion=R.from_euler("xyz", [euler[0], euler[1], euler[2]]).as_quat()
        w,x,y,z=quaternion[3],quaternion[0],quaternion[1],quaternion[2]
        new_quaternion=[x,-y,-z,w]
        rr.log("drone", rr.Transform3D(translation=new_position, rotation=rr.Quaternion(xyzw=new_quaternion)))
        position=struct.pack("ffffff", pos[0], pos[1], pos[2], euler[0], euler[1], euler[2])
        telem_data=sock.sendto(position,(HOST, TELEM_PORT))
        if telem_data == -1:
            logging.error(f"Failed to send data to {address}")
            continue
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
    main()
