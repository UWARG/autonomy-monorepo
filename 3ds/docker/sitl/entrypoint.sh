#!/usr/bin/env bash
set -euo pipefail

: "${SITL_HOME_LAT:=-35.363261}"
: "${SITL_HOME_LON:=149.165230}"
: "${SITL_HOME_ALT:=584}"
: "${SITL_HOME_HEADING:=353}"
: "${SITL_SPEEDUP:=10}"

home="${SITL_HOME_LAT},${SITL_HOME_LON},${SITL_HOME_ALT},${SITL_HOME_HEADING}"

echo "Starting ArduCopter SITL at ${home}; MAVLink TCP is available on port 5762."

# ArduCopter starts SERIAL0 as a blocking console on TCP 5760. Make it
# non-blocking so startup reaches SERIAL1, the MAVLink endpoint used by MAVROS.
# SERIAL1 remains on the standard MAVLink TCP port 5762.
exec /ardupilot/arducopter \
  --model quad \
  --home "${home}" \
  --speedup "${SITL_SPEEDUP}" \
  --defaults /ardupilot/copter.parm \
  --serial0=tcp:5760 \
  --serial1=tcp:5762
