import math
import struct
import time

import numpy as np
import pybullet as p

import constants
import state


class Range_Finder():
    def __init__(self,direction=[0,0,-1],dist=10): #global direction
        self.direction=direction
        self.dist=dist
    
    def update(self):
        pos,orn=p.getBasePositionAndOrientation(state.robot_id)
        self.ray_from=np.array(pos)
        self.ray_to=[
            self.ray_from[0]+self.direction[0]*self.dist,
            self.ray_from[1]+self.direction[1]*self.dist,
            self.ray_from[2]+self.direction[2]*self.dist
        ]
    
        result=p.rayTest(self.ray_from,self.ray_to)
        id=result[0][0] #tuple inside array
        coordinates=result[0][3]
        while id==state.robot_id:
            self.ray_to=self.ray_from+np.array(self.direction)*0.01 #if the ray hits the robot, move the ray slightly forward
            result=p.rayTest(self.ray_from,self.ray_to)
            id=result[0][0]
            coordinates=result[0][3]
        self.range=math.sqrt(
            (coordinates[0]-self.ray_from[0])**2+
            (coordinates[1]-self.ray_from[1])**2+
            (coordinates[2]-self.ray_from[2])**2
        )
    def range_thread(self):
        while True:
            self.update()
            time.sleep(1/constants.RANGE_FINDER_FPS)
            state.groundside_socket.sendto(struct.pack("f",self.range),('127.0.0.1', 8001))
