# A triggered dual-camera image capture with geolocation and orientation data.
# Sent from the drone to groundside upon trigger.

std_msgs/Header header

sensor_msgs/Image forward_image
sensor_msgs/Image downward_image
Coordinate location
sensor_msgs/Imu imu
sensor_msgs/Range range
