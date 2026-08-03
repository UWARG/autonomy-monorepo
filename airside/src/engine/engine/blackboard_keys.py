"""
Central registry of blackboard key names shared between behaviors.
"""

# Lapping deadline, in seconds on the ROS clock
LAPPING_END_TIME_SEC = "lapping_end_time_sec"

# Ordered list of waypoints for the laps
WAYPOINTS = "waypoints"

# Home coordinate (None if not configured)
HOME_WAYPOINT = "home_waypoint"

# The next waypoint index to fly to
WAYPOINT_INDEX = "waypoint_index"

# The waypoint currently being flown to
CURRENT_WAYPOINT = "current_waypoint"

# Duration of the most recent fully-completed lap, in seconds
ESTIMATED_LAP_TIME_SEC = "estimated_lap_time_sec"

# Time in seconds when the current lap started
LATEST_LAP_START_TIME_SEC = "latest_lap_start_time_sec"

# Number of payload items still on board for the dropping sequence
ITEMS_REMAINING = "items_remaining"

# Landing pads not yet claimed for a drop (claiming a pad removes it here)
FREE_LANDING_PADS = "free_landing_pads"

# The landing pad currently being dropped on
TARGET_LANDING_PAD = "target_landing_pad"
