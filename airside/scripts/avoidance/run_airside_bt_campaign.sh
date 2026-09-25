#!/usr/bin/env bash
# Run the fixed airside control matrix and strict ten-attempt wall campaign.
set -uo pipefail

cd "$(dirname "$0")"

artifact_dir="${ARTIFACT_DIR:?ARTIFACT_DIR is required}"
mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"
export ARTIFACT_DIR="$artifact_dir"

python3 -m unittest -v test_synthetic_laserscan.py || exit 1

suite_status=0
control_scenarios="${CONTROL_SCENARIOS:-clear dropout frozen invalid partial pilot_takeover}"
for scenario in $control_scenarios; do
    if ! bash ./run_airside_bt_scenario.sh "$scenario" "$scenario"; then
        suite_status=1
    fi
done

wall_runs="${WALL_RUNS:-10}"
for run_number in $(seq -w 1 "$wall_runs"); do
    if ! bash ./run_airside_bt_scenario.sh wall "wall-run-${run_number}"; then
        suite_status=1
    fi
done

python3 - "$artifact_dir" <<'PY'
import json
import pathlib
import sys

artifact_dir = pathlib.Path(sys.argv[1])
summaries = []
for summary_path in sorted(artifact_dir.glob("*-summary.json")):
    summaries.append(json.loads(summary_path.read_text(encoding="utf-8")))
wall_runs = [item for item in summaries if item["scenario"] == "wall"]
aggregate = {
    "scenario_count": len(summaries),
    "pass_count": sum(item["verdict"] == "PASS" for item in summaries),
    "wall_attempt_count": len(wall_runs),
    "wall_pass_count": sum(item["verdict"] == "PASS" for item in wall_runs),
    "strict_wall_10_of_10": (
        len(wall_runs) == 10
        and all(item["verdict"] == "PASS" for item in wall_runs)
    ),
    "summaries": summaries,
}
(artifact_dir / "airside-bt-aggregate.json").write_text(
    json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
print(json.dumps(aggregate, indent=2, sort_keys=True))
PY

exit "$suite_status"
