#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")"

N=${1:-5}
PASS=0
declare -a RESULTS

for i in $(seq 1 "$N"); do
  echo "=============================================================="
  echo "HOLD SOAK: trial ${i}/${N}"
  echo "=============================================================="
  if (( (i - 1) % 3 == 0 )) && (( i > 1 )); then
    ./sitl_env_reset.sh --daemon || true
  fi
  TS=$(date +%Y%m%d_%H%M%S)
  if ./hold_trial.sh "hold_soak_${TS}_run${i}"; then
    PASS=$((PASS + 1))
    RESULTS[i]="PASS"
  else
    RESULTS[i]="FAIL"
  fi
done

echo "=============================================================="
echo "HOLD SOAK RESULT: ${PASS}/${N} PASS"
for i in $(seq 1 "$N"); do echo "  trial ${i}: ${RESULTS[i]}"; done
[[ "$PASS" -eq "$N" ]]
