import logging
import sys

import pybullet as p


#add primitive objects as well with specified geometry and position
class Object():
    
    def __init__(self,name,position=[0,0,0],orientation=[0,0,0],scale=1, **kwargs):
        orientation=p.getQuaternionFromEuler(orientation)
        self.suffix=name.split(".")[-1]
        self.name=name
        self.position=position
        self.orientation=orientation
        self.scale=scale
        self.kwargs=kwargs
    def _barrel_object(self):
        try:
            radius=self.kwargs["radius"]
            height=self.kwargs["height"]
        except:
            logging.error(f"Barrel object requires radius and length parameters")   
            sys.exit(1)
        barrel_vision=p.createVisualShape(p.GEOM_CYLINDER,radius=radius,length=height)
        barrel_collision=p.createCollisionShape(p.GEOM_CYLINDER,radius=radius,height=height)
        self.id=p.createMultiBody(baseMass=1,baseVisualShapeIndex=barrel_vision,baseCollisionShapeIndex=barrel_collision,basePosition=self.position,baseOrientation=self.orientation)

    def _sphere_object(self):
        try:
            radius=self.kwargs["radius"]
        except:
            logging.error(f"Sphere object requires radius parameter")   
            sys.exit(1)
        sphere_vision=p.createVisualShape(p.GEOM_SPHERE,radius=radius)
        sphere_collision=p.createCollisionShape(p.GEOM_SPHERE,radius=radius)
        self.id=p.createMultiBody(baseMass=1,baseVisualShapeIndex=sphere_vision,baseCollisionShapeIndex=sphere_collision,basePosition=self.position,baseOrientation=self.orientation)


    def _hoop_object(self):
        try:
            radius=self.kwargs["radius"]
        except:
            logging.error(f"Hoop object requires radius parameter")   
            sys.exit(1)
        scale=radius/0.380861 #outer radius of the hoop
        self.id=p.loadURDF("hoop.urdf",self.position,self.orientation,globalScaling=scale) 

    def initialize(self):
        if self.suffix == "urdf":
            self.id=p.loadURDF(self.name,self.position,self.orientation,globalScaling=self.scale)
        elif self.suffix == "sdf":
            self.id=p.loadSDF(self.name,self.position,self.orientation,globalScaling=self.scale)
        else:
            if self.name == "barrel":
                self._barrel_object()
                return
            elif self.name=="sphere":
                self._sphere_object()
                return
            elif self.name=="hoop":
                self._hoop_object()
                return
            logging.error(f"Unknown object type: {self.suffix}")
            sys.exit(1)
