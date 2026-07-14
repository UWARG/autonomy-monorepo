#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")/.."

DRILLS=("$@")
[[ ${#DRILLS[@]} -eq 0 ]] && DRILLS=(lost_target lunge companion_death gps_loss)

up_and_wait_follow() {  # $1 = extra env (e.g. AIRSIDE_LUNGE=true), $2 = artifact dir (container path)
  docker compose down --remove-orphans --timeout 20 >/dev/null 2>&1 || true
  eval "AIRSIDE_AUTO_ARM=true AIRSIDE_WORLD_TARGET=true $1 docker compose up -d" || return 2
  docker compose exec -d airside bash -lc \
    "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
     python3 /monorepo/airside/scripts/flight_recorder.py --out $2/flight.csv" || true
  echo -n "  waiting for 'follow active' (max 300s)"
  for i in $(seq 1 60); do
    if docker compose logs airside 2>/dev/null | grep -q "follow active"; then
      echo " ... following"
      return 0
    fi
    echo -n "."
    sleep 5
  done
  echo " TIMEOUT"
  return 1
}

finish() {  # $1 = artifact dir (host), $2 = drill name, $3 = pass (0/1)
  docker compose logs airside > "$1/airside.log" 2>&1
  docker compose down --timeout 20 >/dev/null 2>&1
  if [[ "$3" -eq 0 ]]; then
    echo "DRILL ${2}: PASS  (${1})"
  else
    echo "DRILL ${2}: FAIL  (${1})"
  fi
  return "$3"
}

overall=0
for drill in "${DRILLS[@]}"; do
  TS=$(date +%Y%m%d_%H%M%S)
  ART_REL="test_artifacts/drill_${drill}_${TS}"
  ART_HOST="$(pwd)/${ART_REL}"
  ART_CONT="/monorepo/airside/${ART_REL}"
  mkdir -p "${ART_HOST}"
  echo "=============================================================="
  echo "DRILL: ${drill}  ->  ${ART_REL}"
  echo "=============================================================="
  pass=1
  case "$drill" in

    lost_target)
      if up_and_wait_follow "" "${ART_CONT}"; then
        sleep 30   # let it converge/settle
        docker compose exec airside bash -lc 'pkill -f sim_target' || true
        sleep 15
        LOG=$(docker compose logs airside 2>/dev/null)
        if grep -q "target lost" <<<"$LOG" && grep -q "requesting LOITER" <<<"$LOG"; then
          pass=0
        fi
      fi
      ;;

    lunge)
      if up_and_wait_follow "AIRSIDE_LUNGE=true" "${ART_CONT}"; then
        echo -n "  waiting for the lunge to trip BRAKE (max 240s)"
        for i in $(seq 1 48); do
          LOG=$(docker compose logs airside 2>/dev/null)
          if grep -q "EmergencyStop ENGAGED" <<<"$LOG" \
             && grep -q "requesting BRAKE" <<<"$LOG"; then
            echo " ... BRAKE latched"
            pass=0
            break
          fi
          echo -n "."
          sleep 5
        done
        [[ $pass -ne 0 ]] && echo " TIMEOUT"
      fi
      ;;

    companion_death)
      if up_and_wait_follow "" "${ART_CONT}"; then
        sleep 20
        docker compose exec airside bash -lc \
          "pkill -f 'lib/engine/manager' || pkill -f '[e]ngine.*manager'" || true
        echo "  engine manager killed; watching the FC hold via the recorder (20s)..."
        sleep 20
        python3 - "$ART_HOST/flight.csv" <<'PYEOF'
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1], newline="")) if r["t"]]
# setpoints stop when the manager dies: find the last fresh setpoint sample
cut = None
for i, r in enumerate(rows):
    if r["sp_age_s"] and float(r["sp_age_s"]) < 0.3:
        cut = i
if cut is None or cut >= len(rows) - 20:
    print("  could not isolate a post-kill window"); sys.exit(1)
post = rows[cut + 1:]
alts = [float(r["z"]) for r in post]
disarmed = any(r["armed"] == "0" for r in post)
drop = max(alts) - min(alts)
print(f"  post-kill: {len(post)} samples, altitude band {drop:.2f} m, disarmed={disarmed}")
sys.exit(0 if (drop < 1.0 and not disarmed) else 1)
PYEOF
        pass=$?
      fi
      ;;

    gps_loss)
      for attempt in 1 2; do
      if up_and_wait_follow "" "${ART_CONT}"; then
        sleep 20
        PRE=$(docker compose logs airside 2>/dev/null)
        if grep -qE "requesting LOITER|requesting BRAKE|-> release" <<<"$PRE"; then
          echo "  precondition lost (early LOITER/BRAKE/release); retrying on a fresh stack"
          docker compose down --timeout 20 >/dev/null 2>&1
          continue
        fi
        MARK=$(wc -l <<<"$PRE")
        echo "  setting SIM_GPS_DISABLE=1 ..."
        if ! docker compose exec airside bash -lc \
          "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
           timeout 25 ros2 service call /mavros/param/set_parameters \
             rcl_interfaces/srv/SetParameters \
             \"{parameters: [{name: 'SIM_GPS_DISABLE', value: {type: 2, integer_value: 1}}]}\" \
           | grep -q 'successful=True'" ; then
          echo "  (service path failed; falling back to the mav param CLI)"
          docker compose exec airside bash -lc \
            "source /opt/ros/humble/setup.bash && source /ros_ws/install/setup.bash && \
             timeout 90 ros2 run mavros mav param set SIM_GPS_DISABLE 1" || true
        fi
        sleep 40
        POST=$(docker compose logs airside 2>/dev/null | tail -n +$((MARK + 1)))
        DEGRADED=false
        grep -qE "released: ekf degraded|action .*-> stream_zero|EKF Failsafe|EKF variance|requesting LOITER" \
          <<<"$POST" && DEGRADED=true
        if $DEGRADED && ! grep -q "requesting BRAKE" <<<"$POST"; then
          pass=0
        else
          echo "  (post-loss log lacked a degradation response; inspect airside.log)"
        fi
      fi
      [[ $pass -eq 0 ]] && break
      done
      ;;

    *)
      echo "unknown drill: ${drill}" >&2
      ;;
  esac
  finish "${ART_HOST}" "${drill}" "${pass}" || overall=1
done

echo "=============================================================="
[[ $overall -eq 0 ]] && echo "GATE 3C DRILLS: ALL PASS" || echo "GATE 3C DRILLS: FAILURES (see above)"
exit $overall
