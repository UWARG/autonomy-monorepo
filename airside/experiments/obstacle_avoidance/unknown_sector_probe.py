"""Record the planner's all-unknown sector semantics for qualification reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from obstacle_avoidance import (
    BendyRuler2D,
    PlanRequest,
    Point2D,
    SectorScan,
    sector_scan_to_snapshot,
)


def run_probe(sector_count: int = 72) -> dict[str, object]:
    """Return the observable all-``None`` adapter and planner result."""

    timestamp_s = 10.0
    scan = SectorScan(
        ranges_m=(None,) * sector_count,
        angle_offset_rad=0.0,
        angle_increment_rad=2.0 * 3.141592653589793 / sector_count,
        timestamp_s=timestamp_s,
        healthy=True,
    )
    snapshot = sector_scan_to_snapshot(
        scan,
        sensor_position=Point2D(0.0, 0.0),
        sensor_heading_rad=0.0,
        obstacle_radius_m=0.75,
    )
    result = BendyRuler2D().plan(
        PlanRequest(
            start=Point2D(0.0, 0.0),
            goal=Point2D(20.0, 0.0),
            obstacles=snapshot,
            now_s=timestamp_s,
        )
    )
    return {
        "input_sector_count": sector_count,
        "input_none_count": sector_count,
        "input_healthy": scan.healthy,
        "snapshot_healthy": snapshot.healthy,
        "snapshot_obstacle_count": len(snapshot.obstacles),
        "planner_status": result.status.value,
        "planner_reason": result.reason.value if result.reason is not None else None,
        "physical_flight_blocker": bool(
            snapshot.healthy
            and not snapshot.obstacles
            and result.status.value == "PATH_FOUND"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()
    payload = run_probe()
    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output_json is not None:
        args.output_json.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
