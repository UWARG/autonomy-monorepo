"""
Central registry of blackboard key names shared between behaviors.
"""

# Number of laps still to be started in the lapping phase
LAPS_REMAINING = "laps_remaining"

# Ordered list of waypoints for the laps
WAYPOINTS = "waypoints"

# Where the drone was armed, at the return altitude; the mission lands here
LAUNCH_POINT = "launch_point"

# The next index to fly to
WAYPOINT_INDEX = "waypoint_index"

# Whether the current lap made it back to the first waypoint
LAP_CLOSED = "lap_closed"

# The waypoint currently being flown to
CURRENT_WAYPOINT = "current_waypoint"

# The lap route around current_waypoint, as (waypoints, index of current_waypoint)
WAYPOINT_PATH = "waypoint_path"

# Time in seconds when the current lap started
LATEST_LAP_START_TIME_SEC = "latest_lap_start_time_sec"

# Number of payload items still on board for the dropping sequence
ITEMS_REMAINING = "items_remaining"

# Landing pads not yet claimed for a drop (claiming a pad removes it here)
FREE_LANDING_PADS = "free_landing_pads"

# The landing pad currently being dropped on
TARGET_LANDING_PAD = "target_landing_pad"
