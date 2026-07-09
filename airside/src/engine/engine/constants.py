"""Tuning constants for the airside lapping behaviors."""

# A new lap is only started if this multiple of the last lap time fits before the deadline.
LAP_TIME_MARGIN = 1.2

# How close (meters) counts as having reached a waypoint.
WAYPOINT_ACCEPTANCE_RADIUS_M = 1.0

# Give up on a waypoint if not reached within this many seconds.
WAYPOINT_NAV_TIMEOUT_S = 120.0

# RC channel that controls the kill switch, pausing the mission.
KILL_SWITCH_RC_CHANNEL = 7
# RC channel that signals the completion of target reconnaissance and starts the land phase.
RECON_COMPLETE_RC_CHANNEL = 8

# PWM value when the RC switch counts as flipped.
RC_SWITCH_HIGH_PWM = 1700

# Relative altitude (meters) to climb to on takeoff.
TAKEOFF_ALTITUDE_M = 15.0

# Takeoff tolerance from target altitude.
TAKEOFF_ALTITUDE_TOLERANCE_M = 1.0
