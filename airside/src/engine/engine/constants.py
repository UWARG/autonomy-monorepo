"""Tuning constants for the airside engine behaviors."""

# ArduPilot flight mode in which the engine is allowed to command the drone.
GUIDED_MODE = "GUIDED"

# Behavior tree tick period, milliseconds.
TICK_PERIOD_MS = 500.0
# Whether or not to print the tree with Unicode characters on every tick.
UNICODE_TREE_DEBUG = False

# Lapping deadline, from engine startup, seconds.
LAPPING_DURATION_SEC = 120.0

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

# A new lap is only started if this multiple of the last lap time fits before the deadline.
LAP_TIME_MARGIN = 1.2

# How close (meters) counts as having reached a waypoint.
WAYPOINT_ACCEPTANCE_RADIUS_M = 1.0

# Give up on a waypoint if not reached within this many seconds.
WAYPOINT_NAV_TIMEOUT_S = 120.0

# Master switch for the RC switch behaviors.
RC_SWITCHES_ENABLED = True

# RC channel that controls the kill switch, pausing the mission.
KILL_SWITCH_RC_CHANNEL = 7
# RC channel that signals the completion of target reconnaissance and starts the land phase.
RECON_COMPLETE_RC_CHANNEL = 6

# PWM value when the RC switch counts as flipped.
RC_SWITCH_HIGH_PWM = 1700

# Relative altitude (meters) to climb to on takeoff.
TAKEOFF_ALTITUDE_M = 15.0

# Relative altitude (meters) above which the drone counts as already flying.
TAKEOFF_AIRBORNE_THRESHOLD_M = 2.0

# Takeoff tolerance from target altitude.
TAKEOFF_ALTITUDE_TOLERANCE_M = 1.0

# Number of payload items on board at the start of the mission.
INITIAL_ITEM_COUNT = 3

# Relative altitude (meters) to climb back to after releasing a payload.
DROP_ASCEND_ALTITUDE_M = 15.0

# Payload release actuates a servo over MAVROS (MAV_CMD_DO_SET_SERVO).
# TODO: confirm the servo output channel and open/closed PWM values
# against the actual drop hardware.
PAYLOAD_RELEASE_SERVO_CHANNEL = 9
PAYLOAD_RELEASE_OPEN_PWM = 1900
PAYLOAD_RELEASE_CLOSED_PWM = 1100
