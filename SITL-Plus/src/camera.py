"""PyBullet camera sensor simulation and Rerun logging."""

import logging
import os
import struct
import time

import cv2
import numpy as np
import pybullet as p
import rerun as rr

import constants
import state

prev_state = state.update
HOST = os.getenv("SENSOR_HOST")
if not HOST:
    raise ValueError("SENSOR_HOST environment variable is not set")

_DOWNWARD_DIRECTIONS = ([0, 0, -1], [0, 0, 1])


class Camera:
    """Simulated camera attached to a PyBullet body."""

    def _get_view_matrix(self):
        pos, orn = p.getBasePositionAndOrientation(self.attached_to_object)
        rot_matrix = p.getMatrixFromQuaternion(orn)
        rot_matrix = np.reshape(rot_matrix, (3, 3))
        if self.direction in _DOWNWARD_DIRECTIONS:
            up_vector = [1, 0, 0]
        else:
            up_vector = [0, 0, 1]
        local_direction = rot_matrix @ np.array(self.direction)
        cam_pos = np.array(pos) + constants.CAMERA_OFFSET * np.array(local_direction)
        self.view_matrix = p.computeViewMatrix(
            cam_pos,
            cam_pos + np.array(local_direction),
            np.array(up_vector),
        )

    def __init__(
        self,
        attached_to_object,
        port,
        fov=60,
        near=1,
        far=100.0,
        height=224,
        width=224,
        direction=None,
        depth_map: bool = True,
    ):
        if direction is None:
            direction = [0, 0, -1]
        self.attached_to_object = attached_to_object
        self.direction = direction
        self.port = port
        self.depth_map = depth_map
        self._get_view_matrix()

        self.fov = fov
        self.height = height
        self.width = width
        self.aspect = width / height
        self.near = near
        self.far = far
        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov, width / height, near, far
        )
        self.rgb_img = None
        self.depth_img = None
        self.seg_img = None

    def update(self):
        """Refresh the camera view matrix."""
        self._get_view_matrix()
        self.projection_matrix = p.computeProjectionMatrixFOV(
            self.fov, self.aspect, self.near, self.far
        )
        time.sleep(1 / constants.CAMERA_FPS)

    def capture_image(self):
        """Capture RGB, depth, and segmentation images from PyBullet."""
        self.rgb_img, self.depth_img, self.seg_img = p.getCameraImage(
            self.width,
            self.height,
            self.view_matrix,
            self.projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )[2:5]

    def camera_thread(self):
        """Capture images and send them over UDP and to Rerun."""
        while True:
            self.update()
            self.capture_image()
            rgba_array = np.reshape(self.rgb_img, (self.width, self.height, 4)).astype(
                np.uint8
            )
            rgb_array = cv2.cvtColor(rgba_array, cv2.COLOR_RGBA2BGR)
            rr.log(str(self.port) + "_rgb_image", rr.Image(rgb_array))
            ok, rgb_bytes = cv2.imencode(
                ".jpg", rgb_array, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            )
            if not ok:
                logging.error("Failed to encode image")
                continue
            rgb_bytes = rgb_bytes.tobytes()
            real_depth = (
                100
                * self.far
                * self.near
                / (self.far - (self.far - self.near) * np.array(self.depth_img))
            )
            depth_array = np.reshape(real_depth, (self.width, self.height)).astype(
                np.uint16
            )
            rr.log(str(self.port) + "_depth_map", rr.DepthImage(0.01 * depth_array))
            if not self.depth_map:
                depth_bytes = b""
            else:
                ok, depth_bytes = cv2.imencode(
                    ".png", np.array(depth_array), [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
                )
                if not ok:
                    logging.error("Failed to encode depth image")
                    continue
                depth_bytes = depth_bytes.tobytes()

            # Segmentation encoding can be added here if needed in the future.
            udp_header = struct.pack(
                "QQff", len(rgb_bytes), len(depth_bytes), self.far, self.near
            )
            state.airside_socket.sendto(
                udp_header + rgb_bytes + depth_bytes, (HOST, self.port)
            )
