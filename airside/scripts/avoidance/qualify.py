#!/usr/bin/env python3
"""Run the formal PR #144 control matrix and strict 10-run campaign."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

CUSTOM_RUN_COUNT = 10
CONTROLS = (
    "clear_guided",
    "wall_guided_wpnav",
    "wall_guided_vel",
    "wall_auto",
    "wall_guided",
)


def run_scenario(
    script_dir: Path,
    artifact_dir: Path,
    scenario: str,
    label: str,
) -> tuple[int, dict[str, Any] | None]:
    env = os.environ.copy()
    env.update(
        {
            "ARTIFACT_DIR": str(artifact_dir),
            "RUN_LABEL": label,
            "SKIP_HARNESS_TESTS": "1",
        }
    )
    result = subprocess.run(
        ["bash", str(script_dir / "run_all.sh"), scenario],
        cwd=script_dir,
        env=env,
        check=False,
    )
    summary_path = artifact_dir / f"{label}-summary.json"
    if not summary_path.exists():
        return result.returncode, None
    return result.returncode, json.loads(summary_path.read_text(encoding="utf-8"))


def control_accepted(
    scenario: str,
    exit_code: int,
    summary: dict[str, Any] | None,
) -> tuple[bool, str]:
    if summary is None:
        return False, "missing summary"
    if scenario == "wall_guided":
        accepted = (
            exit_code == 1
            and summary.get("verdict") == "FAIL"
            and summary.get("breached") is True
        )
        return accepted, "expected wall-breach negative control"
    if scenario == "wall_guided_vel":
        accepted = (
            exit_code == 0
            and summary.get("verdict") == "PASS"
            and summary.get("breached") is False
        )
        limitation = (
            summary.get("goal_reached_at_s") is None
            or not summary.get("clearance_ok", False)
        )
        note = "stop-only limitation" if limitation else "completed"
        return accepted, note
    if scenario == "clear_guided":
        accepted = (
            exit_code == 0
            and summary.get("verdict") == "PASS"
            and summary.get("goal_reached_at_s") is not None
        )
        return accepted, "clear-path goal"
    accepted = (
        exit_code == 0
        and summary.get("verdict") == "PASS"
        and summary.get("breached") is False
        and summary.get("clearance_ok") is True
        and summary.get("goal_reached_at_s") is not None
    )
    return accepted, "wall avoidance and goal"


def custom_accepted(
    exit_code: int,
    summary: dict[str, Any] | None,
) -> tuple[bool, str]:
    if summary is None:
        return False, "missing summary"
    checks = {
        "exit_zero": exit_code == 0,
        "verdict_pass": summary.get("verdict") == "PASS",
        "armed": summary.get("armed") is True,
        "no_failure": not summary.get("failure_reason"),
        "no_worker_failure": summary.get("worker_failure") is None,
        "no_breach": summary.get("breached") is False,
        "clearance": summary.get("clearance_ok") is True,
        "goal": summary.get("goal_reached_at_s") is not None,
        "distance_sensor": summary.get("distance_sensor_rx", 0) > 0,
        "obstacle_tx": summary.get("obstacle_tx_count", 0) > 0,
        "path_found": summary.get("planner_path_found_count", 0) > 0,
        "no_hold": summary.get("planner_hold_count") == 0,
        "final_path": summary.get("planner_status") == "PATH_FOUND",
    }
    failed = [name for name, passed in checks.items() if not passed]
    return not failed, "ok" if not failed else ", ".join(failed)


def write_report(artifact_dir: Path, aggregate: dict[str, Any]) -> None:
    (artifact_dir / "qualification-summary.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# PR #144 SITL Qualification Summary",
        "",
        f"Overall: **{'PASS' if aggregate['overall_pass'] else 'FAIL'}**",
        "",
        "## Controls",
        "",
        "| Scenario | Accepted | Note |",
        "|---|:---:|---|",
    ]
    for result in aggregate["controls"]:
        lines.append(
            f"| `{result['scenario']}` | "
            f"{'YES' if result['accepted'] else 'NO'} | {result['note']} |"
        )
    lines.extend(
        [
            "",
            "## Custom fresh-boot attempts",
            "",
            "| Run | Accepted | Note |",
            "|---:|:---:|---|",
        ]
    )
    for result in aggregate["custom_runs"]:
        lines.append(
            f"| {result['run']:02d} | "
            f"{'YES' if result['accepted'] else 'NO'} | {result['note']} |"
        )
    lines.extend(
        [
            "",
            (
                f"Custom result: **{aggregate['custom_pass_count']}/"
                f"{CUSTOM_RUN_COUNT}**; replacements: **0**."
            ),
            "",
        ]
    )
    (artifact_dir / "qualification-summary.md").write_text(
        "\n".join(lines),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact_dir")
    args = parser.parse_args()
    script_dir = Path(__file__).resolve().parent
    artifact_dir = Path(args.artifact_dir).resolve()
    artifact_dir.mkdir(parents=True, exist_ok=False)
    test_env = os.environ.copy()
    planner_src = script_dir.parents[2] / "obstacle-avoidance" / "src"
    test_env["PYTHONPATH"] = os.pathsep.join(
        value
        for value in (str(planner_src), test_env.get("PYTHONPATH", ""))
        if value
    )

    unit_log = artifact_dir / "harness-unittest.txt"
    with unit_log.open("w", encoding="utf-8") as output:
        unit_result = subprocess.run(
            [
                sys.executable,
                "-m",
                "unittest",
                "discover",
                "-v",
                "-s",
                str(script_dir),
                "-p",
                "test_*.py",
            ],
            cwd=script_dir,
            env=test_env,
            stdout=output,
            stderr=subprocess.STDOUT,
            check=False,
        )
    if unit_result.returncode != 0:
        print(f"harness tests failed; see {unit_log}", flush=True)
        return 1

    probe_result = subprocess.run(
        [
            sys.executable,
            str(script_dir / "unknown_sector_probe.py"),
            "--output-json",
            str(artifact_dir / "unknown-sector-probe.json"),
        ],
        cwd=script_dir,
        env=test_env,
        check=False,
    )
    if probe_result.returncode != 0:
        print("unknown-sector probe failed", flush=True)
        return 1

    control_results: list[dict[str, Any]] = []
    for scenario in CONTROLS:
        label = f"control-{scenario}"
        exit_code, summary = run_scenario(
            script_dir,
            artifact_dir,
            scenario,
            label,
        )
        accepted, note = control_accepted(scenario, exit_code, summary)
        control_results.append(
            {
                "scenario": scenario,
                "accepted": accepted,
                "note": note,
                "exit_code": exit_code,
                "summary": summary,
            }
        )

    custom_results: list[dict[str, Any]] = []
    for run_number in range(1, CUSTOM_RUN_COUNT + 1):
        label = f"wall_custom_2d_{run_number:02d}"
        exit_code, summary = run_scenario(
            script_dir,
            artifact_dir,
            "wall_custom_2d",
            label,
        )
        accepted, note = custom_accepted(exit_code, summary)
        custom_results.append(
            {
                "run": run_number,
                "accepted": accepted,
                "note": note,
                "exit_code": exit_code,
                "summary": summary,
            }
        )

    custom_pass_count = sum(result["accepted"] for result in custom_results)
    aggregate = {
        "controls": control_results,
        "custom_runs": custom_results,
        "custom_attempt_count": CUSTOM_RUN_COUNT,
        "custom_pass_count": custom_pass_count,
        "replacement_count": 0,
        "overall_pass": (
            all(result["accepted"] for result in control_results)
            and custom_pass_count == CUSTOM_RUN_COUNT
        ),
    }
    write_report(artifact_dir, aggregate)
    return 0 if aggregate["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
