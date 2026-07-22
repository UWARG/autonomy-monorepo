"""Plot the event-format follow recorder CSV; output stays in the artifact dir."""

from __future__ import annotations

import csv
import math
import os
import sys


def number(row, key):
    return float(row[key]) if row.get(key) not in (None, "") else math.nan


def main() -> None:
    if len(sys.argv) != 2:
        print("usage: plot_flight.py <artifact-directory>")
        sys.exit(2)
    artifact = sys.argv[1]
    path = os.path.join(artifact, "flight.csv")
    with open(path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        sys.exit("no telemetry rows")
    t0 = float(rows[0]["host_time_s"])

    targets = [row for row in rows if row["event"] == "target"]
    setpoints = [row for row in rows if row["event"] == "setpoint"]
    diagnostics = [row for row in rows if row["event"] == "diagnostic"]
    modes = [row for row in rows if row["event"] == "mode_transition"]

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    target_t = [float(row["host_time_s"]) - t0 for row in targets]
    ranges = [
        math.sqrt(number(row, "x") ** 2 + number(row, "y") ** 2 + number(row, "z") ** 2)
        for row in targets
    ]
    axes[0].plot(target_t, ranges, lw=1)
    axes[0].axhline(2.5, color="orange", linestyle="--", label="standoff")
    axes[0].axhline(1.5, color="red", label="hard-min")
    axes[0].set_ylabel("raw range (m)")
    axes[0].legend()

    setpoint_t = [float(row["host_time_s"]) - t0 for row in setpoints]
    axes[1].plot(setpoint_t, [number(row, "sp_vx") for row in setpoints], label="vx")
    axes[1].plot(setpoint_t, [number(row, "sp_vy") for row in setpoints], label="vy")
    axes[1].plot(setpoint_t, [number(row, "sp_vz") for row in setpoints], label="vz")
    axes[1].set_ylabel("setpoint (m/s)")
    axes[1].legend()

    diagnostic_t = [float(row["host_time_s"]) - t0 for row in diagnostics]
    states = sorted({row["authority_state"] for row in diagnostics})
    state_index = [states.index(row["authority_state"]) for row in diagnostics]
    axes[2].step(diagnostic_t, state_index, where="post", label="authority")
    axes[2].set_yticks(range(len(states)), states)
    for row in modes:
        axes[2].axvline(float(row["host_time_s"]) - t0, color="grey", alpha=0.3)
        axes[2].text(
            float(row["host_time_s"]) - t0,
            max(len(states) - 1, 0),
            row["mode"],
            rotation=90,
            va="top",
        )
    axes[2].set_xlabel("host time (s)")
    axes[2].set_ylabel("authority state")
    for axis in axes:
        axis.grid(True)
    output = os.path.join(artifact, "flight.png")
    figure.tight_layout()
    figure.savefig(output, dpi=110)
    print(f"saved {output}")


if __name__ == "__main__":
    main()
