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
        match self.suffix:
            case "urdf":
                self.id=p.loadURDF(self.name,self.position,self.orientation,globalScaling=self.scale)
            case "sdf":
                self.id=p.loadSDF(self.name,self.position,self.orientation,globalScaling=self.scale)
            case _:
                logging.error(f"Unknown object type: {self.suffix}")
                sys.exit(1)
