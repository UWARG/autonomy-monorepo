"""Tuning constants for the airside lapping behaviors."""

# A new lap is only started if this multiple of the last lap time fits before the deadline.
LAP_TIME_MARGIN = 1.2

# How close (meters) counts as having reached a waypoint.
WAYPOINT_ACCEPTANCE_RADIUS_M = 1.0

# Give up on a waypoint if not reached within this many seconds.
WAYPOINT_NAV_TIMEOUT_S = 120.0