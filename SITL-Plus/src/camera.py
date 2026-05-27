import logging
import struct
import time

import cv2
import numpy as np
import pybullet as p

import constants
import state


class Camera():

    def _get_view_matrix(self):
        pos,orn=p.getBasePositionAndOrientation(self.attached_to_object)
        R=p.getMatrixFromQuaternion(orn)
        R=np.reshape(R,(3,3))
        x=R[:,0] #forward = +x
        y=R[:,1] #left = +y
        z=R[:,2] #up = +z
        
        self.view_matrix=p.computeViewMatrix(np.array(pos)+np.array(x), #camera position
        np.array(pos)+2*np.array(x), # look at
        np.array(z)) #up vector


    def __init__(self,attached_to_object,fps=60,fov=60,near=0.1,far=100.0,height=224,width=224,camera_orientation=[0,0,1]):
        self.attached_to_object=attached_to_object
        self.fps=fps

        self._get_view_matrix()


        self.fov=fov
        self.height=height
        self.width=width
        self.aspect=width/height
        self.near=near
        self.far=far
        self.projection_matrix=p.computeProjectionMatrixFOV(fov,width/height,near,far)
        self.rgb_img=None
        self.depth_img=None
        self.seg_img=None
    def update(self):
        time.sleep(1/constants.CAMERA_FPS)
        self._get_view_matrix()
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
            state.groundside_socket.sendto(udp_header+rgb_bytes+depth_bytes,('127.0.0.1', 8000))
