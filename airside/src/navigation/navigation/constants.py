"""Tuning constants for the position controller's obstacle avoidance."""

# Extra horizontal separation (meters) added on top of every obstacle's
# reported keep-away distance, on all sides.
KEEP_AWAY_MARGIN_M = 5.0

# The keep-away zone is stretched along the obstacle's direction of travel by
# its speed times this many seconds, so the side it is flying towards is kept
# clearer than the side it is leaving. Obstacles are frozen at their last
# report, so this should at least cover the time between two reports.
FORWARD_ZONE_LOOKAHEAD_S = 5.0

# Upper limit (meters) on that forward stretch, however fast the obstacle is.
FORWARD_ZONE_MAX_EXTENSION_M = 100.0

# Below this speed (meters per second) an obstacle counts as stationary: its
# direction is meaningless, so its zone is not stretched.
FORWARD_ZONE_MIN_SPEED_MPS = 0.5

# Detours run between corner points placed around each keep-away zone. This is
# how many corners a zone gets; more corners hug the zone tighter but make
# planning slower. Must be a multiple of 4.
ZONE_CORNER_COUNT = 12

# How far outside the edge of its zone (meters) the corners are placed, so a
# drone following a detour is not skimming the zone itself.
ZONE_CORNER_STANDOFF_M = 1.0

# The drone keeps going round an obstacle on the side it already chose unless
# the other side is at least this much shorter (meters). Stops it from flipping
# between left and right when both are about as long.
SIDE_SWITCH_PENALTY_M = 10.0

# A detour setpoint is pushed past the corner it is heading for, up to this far
# from the drone (meters) where that is clear, so the drone does not slow down
# for every corner. The path is re-planned before it gets there.
DETOUR_SETPOINT_LEAD_M = 30.0

# How far away (meters) the setpoint is placed when flying out of a zone the
# drone is already inside.
ESCAPE_DISTANCE_M = 30.0
