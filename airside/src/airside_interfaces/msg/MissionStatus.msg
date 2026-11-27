# Engine mission progress for the ground station, published latched on
# engine/status after every behaviour tree tick.

# STARTING, PAUSED, RUNNING, ABORTED or COMPLETE
string state
# Mission step running, e.g. "Takeoff", "Lapping", "TargetReconnaissance"
string phase
# Deepest running behaviour, e.g. "FlyToWaypoint"
string behaviour
# Current lap (1-based) and the total; 0 outside lapping
uint8 lap
uint8 lap_count
# Lap waypoint being flown to (1-based) and the total; 0 when none
uint8 waypoint
uint8 waypoint_count
# Latest progress message sent to the ground as STATUSTEXT
string last_message
