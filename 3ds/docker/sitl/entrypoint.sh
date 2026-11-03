#!/usr/bin/env bash
set -euo pipefail

: "${SITL_HOME_LAT:=-35.363261}"
: "${SITL_HOME_LON:=149.165230}"
: "${SITL_HOME_ALT:=584}"
: "${SITL_HOME_HEADING:=353}"
: "${SITL_SPEEDUP:=1}"

home="${SITL_HOME_LAT},${SITL_HOME_LON},${SITL_HOME_ALT},${SITL_HOME_HEADING}"

echo "Starting ArduCopter SITL at ${home}; MAVLink TCP is available on port 5762."

# SERIAL1 is the standard MAVLink telemetry port. The :wait suffix prevents
# the simulated vehicle from advancing before MAVROS establishes its connection.
exec /ardupilot/arducopter \
  --model quad \
  --home "${home}" \
  --speedup "${SITL_SPEEDUP}" \
  --defaults /ardupilot/copter.parm \
  --serial1=tcp:*:5762:wait
