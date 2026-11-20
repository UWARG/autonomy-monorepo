"""
CameraIMU - Data class for a single IMU reading from Oak-D camera.
"""

from dataclasses import dataclass, field
import time


@dataclass
class CameraIMU:
    """Data Class for a single IMU sample (accelerometer + gyroscope + magnetometer)"""

    accel_x: float
    accel_y: float
    accel_z: float

    gyro_x: float
    gyro_y: float
    gyro_z: float

    mag_x: float
    mag_y: float
    mag_z: float

    timestamp: float = field(default_factory=time.time)
