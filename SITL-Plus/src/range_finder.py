import math
import struct
import time

import numpy as np
import pybullet as p
import os

import constants
import state
import rerun as rr
prev_state=state.update
HOST=os.getenv("SENSOR_HOST")
if not HOST:
    raise ValueError("SENSOR_HOST environment variable is not set")


class Range_Finder():
    def __init__(self,port,direction=[0,0,-1],dist=100):
        self.port=port
        self.direction=direction
        self.dist=dist
    
    def update(self):
        pos,orn=p.getBasePositionAndOrientation(state.robot_id)
        ray_from=np.array(pos)
        R=p.getMatrixFromQuaternion(orn)
        R=np.reshape(R,(3,3))
        local_direction=R@np.array(self.direction)/np.linalg.norm(R@np.array(self.direction))
        ray_to=ray_from+local_direction*self.dist
    
        result=p.rayTest(ray_from,ray_to)
        id=result[0][0] #tuple inside array
        coordinates=result[0][3]
        while id==state.robot_id:
            ray_from=ray_from+np.array(local_direction)*0.01 #if the ray hits the robot, move the ray slightly forward
            result=p.rayTest(ray_from,ray_to)
            id=result[0][0]
            coordinates=result[0][3]
        self.range=math.sqrt(
            (coordinates[0]-ray_from[0])**2+
            (coordinates[1]-ray_from[1])**2+
            (coordinates[2]-ray_from[2])**2
        )
    def range_thread(self):
        while True:
            time.sleep(1/constants.RANGE_FINDER_FPS)
            self.update()
            state.airside_socket.sendto(struct.pack("f",self.range),(HOST, self.port)) # 127.0.0.1 is the localhost address 172.17.0.1 is the docker container address
            rr.log(str(self.port) + "_range", rr.Scalars(self.range))
