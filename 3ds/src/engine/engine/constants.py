"""Tuning constants for the 3DS engine behaviors."""

# ArduPilot flight mode in which the engine is allowed to command the drone.
GUIDED_MODE = "GUIDED"

# Behavior tree tick period, milliseconds.
TICK_PERIOD_MS = 500.0
# Whether or not to print the tree with Unicode characters on every tick.
UNICODE_TREE_DEBUG = False

# MAVLink message IDs the engine needs streamed from the FCU.
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_RC_CHANNELS = 65

# Per-message stream rates requested from ArduPilot (message ID -> Hz).
STREAM_RATE_REQUESTS_HZ = {
    MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10.0,
    MAVLINK_MSG_ID_RC_CHANNELS: 5.0,
}

# Baseline rate for all legacy streams (REQUEST_DATA_STREAM fallback), Hz.
BASELINE_STREAM_RATE_HZ = 4

# How close (meters) counts as having reached a waypoint.
WAYPOINT_ACCEPTANCE_RADIUS_M = 1.0

# Give up on a waypoint if not reached within this many seconds.
WAYPOINT_NAV_TIMEOUT_S = 120.0

# Setpoints below this relative altitude (meters) are rejected.
SETPOINT_MIN_ALTITUDE_M = 2.0

# Give up on a photo if the camera has not saved it within this many seconds.
PHOTO_CAPTURE_TIMEOUT_S = 10.0

# Master switch for the RC switch behaviors.
RC_SWITCHES_ENABLED = True

# RC channel that controls the kill switch, pausing the mission.
KILL_SWITCH_RC_CHANNEL = 7

# PWM value when the RC switch counts as flipped.
RC_SWITCH_HIGH_PWM = 1700

# Relative altitude (meters) to climb to on takeoff.
TAKEOFF_ALTITUDE_M = 15.0

# Relative altitude (meters) above which the drone counts as already flying.
TAKEOFF_AIRBORNE_THRESHOLD_M = 2.0

# Takeoff tolerance from target altitude.
TAKEOFF_ALTITUDE_TOLERANCE_M = 1.0
