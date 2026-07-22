#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")/.."

COMPOSE=(docker compose -f compose.follow.sitl.yaml)
DRILLS=("$@")
[[ ${#DRILLS[@]} -eq 0 ]] && \
  DRILLS=(rc_kill mode_land mode_loiter stale_state lost_target lunge crossing)

ros() {
  "${COMPOSE[@]}" exec -T airside bash -lc \
    "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && $1"
}

up_and_enable() { # lunge, crossing, container artifact directory
  local lunge="$1" crossing="$2" artifact="$3"
  "${COMPOSE[@]}" down --remove-orphans --timeout 20 >/dev/null 2>&1 || true
  AIRSIDE_LUNGE="${lunge}" AIRSIDE_CROSSING="${crossing}" \
    "${COMPOSE[@]}" up -d || return 2
  "${COMPOSE[@]}" exec -d airside bash -lc \
    "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
     python3 /monorepo/airside/scripts/flight_recorder.py --duration 600 --out ${artifact}/flight.csv" \
    || return 2

  echo -n "  waiting for armed GUIDED + fresh target (max 240s)"
  for _ in $(seq 1 48); do
    state=$(ros "timeout 5 ros2 topic echo /mavros/state --once" 2>/dev/null || true)
    target=$(ros "timeout 5 ros2 topic echo /perception/target --once" 2>/dev/null || true)
    if grep -q "mode: GUIDED" <<<"${state}" && grep -q "armed: true" <<<"${state}" \
       && grep -q "track_id:" <<<"${target}"; then
      echo " ... ready"
      break
    fi
    echo -n "."
    sleep 5
  done
  response=$(ros "ros2 service call /follow/set_enabled std_srvs/srv/SetBool '{data: true}'" \
    2>/dev/null || true)
  grep -q "success=True" <<<"${response}" || {
    echo " enable rejected: ${response}"; return 1;
  }
  for _ in $(seq 1 20); do
    "${COMPOSE[@]}" logs airside 2>/dev/null | grep -q "state=active" && return 0
    sleep 1
  done
  echo "  follow did not become active"
  return 1
}

set_mode() {
  ros "ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: $1}'" \
    >/dev/null 2>&1
}

finish() { # host artifact, drill, pass
  local artifact="$1" drill="$2" pass="$3"
  "${COMPOSE[@]}" logs airside > "${artifact}/airside.log" 2>&1
  "${COMPOSE[@]}" down --timeout 20 >/dev/null 2>&1
  if [[ "${pass}" -eq 0 ]]; then
    echo "DRILL ${drill}: PASS (${artifact})"
  else
    echo "DRILL ${drill}: FAIL (${artifact})"
  fi
  return "${pass}"
}

overall=0
for drill in "${DRILLS[@]}"; do
  timestamp=$(date +%Y%m%d_%H%M%S)
  relative="test_artifacts/drill_${drill}_${timestamp}"
  host="$(pwd)/${relative}"
  container="/monorepo/airside/${relative}"
  mkdir -p "${host}"
  echo "==================== ${drill} ===================="
  pass=1
  lunge=false
  crossing=false
  [[ "${drill}" == "lunge" ]] && lunge=true
  [[ "${drill}" == "crossing" ]] && crossing=true

  if up_and_enable "${lunge}" "${crossing}" "${container}"; then
    case "${drill}" in
      rc_kill)
        ros "ros2 topic pub --rate 20 --times 10 /mavros/rc/in mavros_msgs/msg/RCIn \
          '{channels: [1500,1500,1500,1500,1500,1500,1900,1000]}'" >/dev/null
        sleep 2
        # Returning kill low cannot resume. A deliberate CH8 low->high edge can.
        before=$("${COMPOSE[@]}" logs airside 2>/dev/null | grep -c "state=active" || true)
        ros "ros2 topic pub --rate 20 --times 5 /mavros/rc/in mavros_msgs/msg/RCIn \
          '{channels: [1500,1500,1500,1500,1500,1500,1000,1000]}'" >/dev/null
        sleep 1
        middle=$("${COMPOSE[@]}" logs airside 2>/dev/null | grep -c "state=active" || true)
        ros "ros2 topic pub --rate 20 --times 5 /mavros/rc/in mavros_msgs/msg/RCIn \
          '{channels: [1500,1500,1500,1500,1500,1500,1000,1900]}'" >/dev/null
        sleep 2
        after=$("${COMPOSE[@]}" logs airside 2>/dev/null | grep -c "state=active" || true)
        [[ "${middle}" -eq "${before}" && "${after}" -gt "${middle}" ]] && pass=0
        ;;
      mode_land)
        set_mode LAND
        sleep 6
        if grep -q "reason=mode_exit" <("${COMPOSE[@]}" logs airside 2>/dev/null) \
           && python3 scripts/assert_setpoint_release.py "${host}/flight.csv" --mode LAND; then
          pass=0
        fi
        ;;
      mode_loiter)
        set_mode LOITER
        sleep 6
        if grep -q "reason=mode_exit" <("${COMPOSE[@]}" logs airside 2>/dev/null) \
           && python3 scripts/assert_setpoint_release.py "${host}/flight.csv" --mode LOITER; then
          pass=0
        fi
        ;;
      stale_state)
        ros "pkill -f '[m]avros_node'" >/dev/null 2>&1 || true
        sleep 6
        if grep -q "reason=fc_state_stale" <("${COMPOSE[@]}" logs airside 2>/dev/null) \
           && python3 scripts/assert_setpoint_release.py "${host}/flight.csv" \
             --reason fc_state_stale; then
          pass=0
        fi
        ;;
      lost_target)
        ros "pkill -f '[s]im_target'" >/dev/null 2>&1 || true
        sleep 4
        log=$("${COMPOSE[@]}" logs airside 2>/dev/null)
        grep -q "reason=target_lost" <<<"${log}" \
          && grep -q "requesting LOITER" <<<"${log}" && pass=0
        ;;
      lunge)
        echo -n "  waiting for raw-range/closing-rate BRAKE"
        for _ in $(seq 1 60); do
          log=$("${COMPOSE[@]}" logs airside 2>/dev/null)
          if grep -q "reason=proximity_emergency" <<<"${log}" \
             && grep -q "requesting BRAKE" <<<"${log}"; then
            pass=0; echo " ... latched"; break
          fi
          echo -n "."; sleep 2
        done
        ;;
      crossing)
        # A closer candidate with another ID must not replace the active target.
        sleep 2
        diagnostic=$(ros "timeout 5 ros2 topic echo /follow/diagnostics --once" 2>/dev/null || true)
        grep -A1 "key: lock_id" <<<"${diagnostic}" | grep -q "value: '1'" && pass=0
        ;;
      *) echo "unknown drill: ${drill}" ;;
    esac
  fi

  finish "${host}" "${drill}" "${pass}" || overall=1
done

[[ "${overall}" -eq 0 ]] && echo "FOLLOW SITL DRILLS: ALL PASS" \
  || echo "FOLLOW SITL DRILLS: FAILURES"
exit "${overall}"
