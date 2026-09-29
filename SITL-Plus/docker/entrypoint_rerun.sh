#!/usr/bin/env bash

ARDUPILOT_DIR="/app/src/ardupilot"
LOG_DIR="/app/logs"

mkdir -p "$LOG_DIR"
export PYTHONUNBUFFERED=1


export SIM_RATE_HZ=${SIM_RATE_HZ:-800}


echo "[entrypoint] Starting ArduPilot SITL (sim_vehicle.py)..."
cd "$ARDUPILOT_DIR"


LAT=-35.362938
LON=149.165085
ALT=584.0805053710938
DIR=270

source /home/devuser/venv-ardupilot/bin/activate
PHYSICS_IP=$(python3 -c 'import socket, sys; print(socket.gethostbyname(sys.argv[1]))' "${PHYSICS_HOST:-physics}")
env -u DISPLAY python3 -u ./Tools/autotest/sim_vehicle.py -N -v ArduCopter \
-f quad --model JSON:${PHYSICS_IP} -w \
--param SIM_RATE_HZ=${SIM_RATE_HZ} \
--param FRAME_CLASS=1 \
--param FRAME_TYPE=1 \
--out tcpin:0.0.0.0:5761 \
--out host.docker.internal:${GCS_PORT:-14550} \
--mavproxy-args "--non-interactive --moddebug=3 --show-errors --state-basedir=${LOG_DIR}" \
--custom-location=${LAT},${LON},${ALT},${DIR} \
> "${LOG_DIR}/sim_vehicle.log" 2>&1 < /dev/null &
SITL_PID=$!
trap 'kill "$SITL_PID" 2>/dev/null || true' EXIT
trap 'exit 143' TERM
trap 'exit 130' INT
wait "$SITL_PID"
