""" 
Constants used throughout the Airside system. 
"""
import math 

# General 
UINT16_MAX = 65535

# MAVLink Connection
MAVLINK_TCP_HOST = "127.0.0.1"
MAVLINK_TCP_PORT = 5760

# Camera FOV (radians) used to project clicked pixels into metric offsets
CAMERA_HFOV_RAD = math.radians(80)
CAMERA_VFOV_RAD = math.radians(55)

ARDU_CAMERA_VFOV_RAD = math.radians(55)
ARDU_CAMERA_HFOV_RAD = math.radians(80)