#!/usr/bin/env bash
# Run the fixed airside control matrix and strict ten-attempt wall campaign.
set -uo pipefail

cd "$(dirname "$0")"

final_artifact_dir="${ARTIFACT_DIR:?ARTIFACT_DIR is required}"
mkdir -p "$final_artifact_dir"
final_artifact_dir="$(realpath "$final_artifact_dir")"

# ROS bag and JSONL flushes can briefly stall callbacks when the destination is
# a Windows-mounted WSL path. Formal runs may stage on the Linux filesystem and
# copy the complete (or partial, on failure) evidence to the requested location.
artifact_dir="${STAGING_ARTIFACT_DIR:-$final_artifact_dir}"
mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"
if [ "$artifact_dir" != "$final_artifact_dir" ] \
    && [ -n "$(find "$artifact_dir" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
    echo "staging artifact directory must be empty: $artifact_dir" >&2
    exit 1
fi

sync_artifacts() {
    if [ "$artifact_dir" != "$final_artifact_dir" ]; then
        cp -a "$artifact_dir"/. "$final_artifact_dir"/
    fi
}

finish() {
    status=$?
    trap - EXIT
    if ! sync_artifacts; then
        status=1
    fi
    exit "$status"
}
trap finish EXIT

export ARTIFACT_DIR="$artifact_dir"

planner_src="$(cd ../../../obstacle-avoidance && pwd)/src"
export PYTHONPATH="$planner_src${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -v -s . -p 'test_*.py' || exit 1
python3 unknown_sector_probe.py \
    --output-json "$artifact_dir/unknown-sector-probe.json" || exit 1

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
