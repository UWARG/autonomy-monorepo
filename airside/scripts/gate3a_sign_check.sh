#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")/.."
COMPOSE=(docker compose -f compose.follow.sitl.yaml)

TS=$(date +%Y%m%d_%H%M%S)
ART="$(pwd)/test_artifacts/sign_check_${TS}"
mkdir -p "${ART}"

"${COMPOSE[@]}" down --remove-orphans --timeout 20 >/dev/null 2>&1 || true
AIRSIDE_FOLLOW_LAUNCH=follow_hitl.launch.py AIRSIDE_WORLD_TARGET=false \
  "${COMPOSE[@]}" up -d || exit 2

echo -n "[3a] waiting for MAVROS<->FCU heartbeat (max 180s)"
for i in $(seq 1 36); do
  if "${COMPOSE[@]}" logs airside 2>/dev/null | grep -q "Got HEARTBEAT"; then
    echo " ... connected"
    break
  fi
  echo -n "."
  sleep 5
done

echo "[3a] requesting GUIDED (disarmed; retries until the EKF lets it in) ..."
"${COMPOSE[@]}" exec airside bash -lc '
  source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash
  for i in $(seq 1 30); do
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: GUIDED}" >/dev/null 2>&1
    MODE=$(timeout 5 ros2 topic echo /mavros/state --once 2>/dev/null | grep -m1 "^mode:" | awk "{print \$2}")
    echo "  mode=${MODE:-?} (attempt $i)"
    [[ "$MODE" == "GUIDED" ]] && exit 0
    sleep 5
  done
  echo "FC never accepted GUIDED (EKF not ready?)"; exit 1
' | tee "${ART}/guided.log"
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
  "${COMPOSE[@]}" logs airside > "${ART}/airside.log" 2>&1
  "${COMPOSE[@]}" down --timeout 20 >/dev/null 2>&1
  echo "GATE 3A: FAIL (could not enter GUIDED; see ${ART})"
  exit 1
fi

echo "[3a] running sign_check.py..."
"${COMPOSE[@]}" exec airside bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
   python3 /monorepo/airside/scripts/sign_check.py" | tee "${ART}/sign_check.log"
RC=${PIPESTATUS[0]}

"${COMPOSE[@]}" logs airside > "${ART}/airside.log" 2>&1
"${COMPOSE[@]}" down --timeout 20 >/dev/null 2>&1

if grep -q "SIGN-AND-MASK GATE: PASS" "${ART}/sign_check.log"; then
  echo "GATE 3A: PASS  (${ART})"
  exit 0
fi
echo "GATE 3A: FAIL (rc=${RC}; see ${ART})"
exit 1
