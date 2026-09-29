"""
Central registry of blackboard key names shared between behaviors.
"""

# Where the drone was armed, at the return altitude; the mission lands here
LAUNCH_POINT = "launch_point"

# The waypoint currently being flown to (a received setpoint, or the launch point)
CURRENT_WAYPOINT = "current_waypoint"
