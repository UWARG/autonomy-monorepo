"""Replay measured HITL timing and select the highest passing follow speed."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import replace

from sim.monte_carlo import soak
from stack_config import DEPLOYED


def stack_from_timing(timing: dict, v_max: float):
    reaction = float(timing["reaction_time_s"])
    return replace(
        DEPLOYED,
        follow=replace(DEPLOYED.follow, v_max=v_max),
        safety=replace(DEPLOYED.safety, reaction_time_s=reaction),
        reflex=replace(DEPLOYED.reflex, reaction_time_s=reaction),
        target_freshness_s=float(timing["target_freshness_s"]),
        stream_hz=float(timing["stream_hz"]),
        ema_alpha=float(timing["ema_alpha"]),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("timing_json")
    parser.add_argument("--episodes", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--outdir", default="sim_output/measured")
    parser.add_argument("--json-out", required=True)
    args = parser.parse_args()
    with open(args.timing_json, encoding="utf-8") as handle:
        timing = json.load(handle)
    if (
        float(timing.get("detector_p05_fps", timing.get("p05_fps", 0.0))) < 10.0
        or float(
            timing.get(
                "detector_capture_to_ros_p99_s",
                timing.get("capture_to_receive_p99_s", float("inf")),
            )
        )
        > 0.300
    ):
        raise SystemExit(
            "measured perception failed Gate 5 (p05 FPS <10 or p99 latency >300 ms); "
            "do not retune simulation around it"
        )

    passing = []
    summaries = {}
    # Search highest-first; retain all results so regressions are explainable.
    for tenth in range(15, 4, -1):
        v_max = tenth / 10.0
        stack = stack_from_timing(timing, v_max)
        run_dir = os.path.join(args.outdir, f"vmax_{v_max:.1f}")
        passed, summary, _ = soak(
            args.episodes,
            args.seed,
            outdir=run_dir,
            stack=stack,
            timing=timing,
        )
        summaries[f"{v_max:.1f}"] = summary
        if passed:
            passing.append(v_max)
            break

    result = {
        "episodes_per_configuration": args.episodes,
        "passing_vmax_mps": passing,
        "selected_vmax_mps": max(passing) if passing else None,
        "summaries": summaries,
    }
    with open(args.json_out, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if passing and max(passing) >= 0.5 else 1)


if __name__ == "__main__":
    main()
