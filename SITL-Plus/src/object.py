import logging
import sys

import pybullet as p


#add primitive objects as well with specified geometry and position
class Object():
    def __init__(self,name,position=[0,0,0],orientation=[0,0,0],scale=1):
        orientation=p.getQuaternionFromEuler(orientation)
        self.suffix=name.split(".")[-1]
        self.name=name
        self.position=position
        self.orientation=orientation
        self.scale=scale
    def initialize(self):
        if self.suffix == "urdf":
            self.id=p.loadURDF(self.name,self.position,self.orientation,globalScaling=self.scale)
        elif self.suffix == "sdf":
            self.id=p.loadSDF(self.name,self.position,self.orientation,globalScaling=self.scale)
        else:
            logging.error(f"Unknown object type: {self.suffix}")
            sys.exit(1)
