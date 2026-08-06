#!/usr/bin/env bash

SITL_PLUS_DIR="/app/src"
ARDUPILOT_DIR="/app/src/ardupilot"
LOG_DIR="/app/logs"

mkdir -p "$LOG_DIR"
export PYTHONUNBUFFERED=1


export SIM_RATE_HZ=800


echo "[entrypoint] Logs: ${LOG_DIR}/sim_vehicle.log (MAVProxy/SITL), ${LOG_DIR}/pybullet.log"

echo "[entrypoint] Starting PyBullet physics (main.py)..."
cd "$SITL_PLUS_DIR"
# Close stdin for physics only — it does not need the TTY. Leave SITL/MAVProxy
# on the compose-provided TTY (stdin_open/tty) so MAVProxy does not see EOF and exit.
uv run python3 main.py >"${LOG_DIR}/pybullet.log" 2>&1 < /dev/null &
MAIN_PID=$!


sleep 10

echo "[entrypoint] Starting ArduPilot SITL (sim_vehicle.py)..."
cd "$ARDUPILOT_DIR"


LAT=-35.362938
LON=149.165085
ALT=584.0805053710938
DIR=270

source /home/devuser/venv-ardupilot/bin/activate
env -u DISPLAY python3 -u ./Tools/autotest/sim_vehicle.py -N -v ArduCopter \
-f quad --model JSON:127.0.0.1 -w \
--param SIM_RATE_HZ=800 \
--param FRAME_CLASS=1 \
--param FRAME_TYPE=1 \
--out tcpin:0.0.0.0:5761 \
--out host.docker.internal:14550 \
--mavproxy-args "--non-interactive --moddebug=3 --show-errors --state-basedir=${LOG_DIR}" \
--custom-location=${LAT},${LON},${ALT},${DIR} \
> "${LOG_DIR}/sim_vehicle.log" 2>&1 < /dev/null &
SITL_PID=$!


# If SITL dies alone, exit so the container does not look "up" with a dead FC.
wait -n "$MAIN_PID" "$SITL_PID"
EXIT_CODE=$?

echo "[entrypoint] Stopping processes..."
kill "$MAIN_PID" "$SITL_PID" 2>/dev/null || true
exit $EXIT_CODE
