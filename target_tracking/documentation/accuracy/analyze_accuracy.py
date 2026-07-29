import argparse
import csv
import pathlib
import statistics

# Tolerance band
IDEAL_PCT = 5.0
PASS_PCT = 10.0
NOISE_STD_MM = 50.0
AXES = ("x", "y", "z")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Summarize accuracy-log XYZ error.")
    parser.add_argument("csv_path", type=pathlib.Path, help="Path to an accuracy_log CSV.")
    parser.add_argument(
        "--plots",
        action="store_true",
        help="Save a Z-over-time PNG per position next to the CSV (requires matplotlib).",
    )
    return parser.parse_args()


def load_rows(csv_path: pathlib.Path) -> list[dict[str, str]]:
    """Read every row of the CSV into a list of dictionaries."""
    with open(csv_path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def ground_truth(row: dict[str, str]) -> tuple[float, float, float]:
    """Return ground-truth (x, y, z) mm, supporting new gt_*_mm and legacy test_distance."""
    if "gt_z_mm" in row:
        return (float(row["gt_x_mm"]), float(row["gt_y_mm"]), float(row["gt_z_mm"]))
    distance_mm = float(row["test_distance"].lower().replace("m", "")) * 1000.0
    return (0.0, 0.0, distance_mm)


def measured(row: dict[str, str], prefix: str) -> tuple[float, float, float]:
    keys = tuple(f"{prefix}_{axis}_mm" for axis in AXES)
    if all(key in row for key in keys):
        return (float(row[keys[0]]), float(row[keys[1]]), float(row[keys[2]]))
    return (float(row["x_mm"]), float(row["y_mm"]), float(row["z_mm"]))


def verdict(abs_err_mm: float, range_mm: float) -> str:
    if range_mm <= 0:
        return "n/a"
    pct = 100.0 * abs_err_mm / range_mm
    if pct <= IDEAL_PCT:
        return "IDEAL"
    if pct <= PASS_PCT:
        return "PASS"
    return "FAIL"


def format_axis(
    name: str,
    gt_val: float,
    cal_vals: list[float],
    raw_vals: "list[float] | None",
    range_mm: float,
) -> str:
    median_cal = statistics.median(cal_vals)
    std_cal = statistics.pstdev(cal_vals)
    err = median_cal - gt_val
    pct = 100.0 * abs(err) / range_mm if range_mm > 0 else 0.0
    line = (
        f"  {name.upper()}: median={median_cal:8.1f} std={std_cal:7.1f} gt={gt_val:7.0f} "
        f"err={err:+8.1f}mm ({pct:5.1f}%) {verdict(abs(err), range_mm)}"
    )
    if raw_vals is not None:
        line += f"   [raw err={statistics.median(raw_vals) - gt_val:+.0f}mm]"
    if std_cal > NOISE_STD_MM:
        line += "  <-- NOISY: not a still single subject, recapture"
    return line


def save_plots(csv_path: pathlib.Path, groups: dict) -> None:
    try:
        import matplotlib  # pylint: disable=import-outside-toplevel

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel
    except ImportError:
        print("matplotlib not installed; skipping plots. Install with: pip install matplotlib")
        return
    for (gt_x, gt_y, gt_z), grp in groups.items():
        zs = [measured(row, "cal")[2] for row in grp]
        fig, axis = plt.subplots()
        axis.plot(range(len(zs)), zs, label="measured Z")
        axis.axhline(gt_z, color="r", linestyle="--", label=f"truth {gt_z:.0f}mm")
        axis.set_title(f"Z over time @ gt(x={gt_x:.0f}, y={gt_y:.0f}, z={gt_z:.0f}) mm")
        axis.set_xlabel("frame")
        axis.set_ylabel("Z (mm)")
        axis.legend()
        out_path = csv_path.parent / f"z_gt_{int(gt_z)}mm.png"
        fig.savefig(out_path)
        plt.close(fig)
        print("wrote", out_path)


def main() -> int:
    """Print a per-position accuracy summary for the CSV given on the command line."""
    args = parse_args()
    rows = load_rows(args.csv_path)
    if not rows:
        print(f"No data rows in {args.csv_path}")
        return -1

    has_raw = all(f"raw_{axis}_mm" in rows[0] for axis in AXES)
    has_gt_xy = "gt_z_mm" in rows[0]

    groups: dict[tuple[float, float, float], list[dict[str, str]]] = {}
    for row in rows:
        groups.setdefault(ground_truth(row), []).append(row)

    print(f"\n{args.csv_path}: {len(rows)} rows, {len(groups)} position(s)")
    print(f"Tolerance vs range(Z):  IDEAL <= {IDEAL_PCT:.0f}%   PASS <= {PASS_PCT:.0f}%")
    if not has_gt_xy:
        print("(legacy log: only distance/Z ground truth known; X/Y skipped, Z is raw)")
    print()

    for (gt_x, gt_y, gt_z), grp in sorted(groups.items(), key=lambda item: item[0][2]):
        cal = [measured(row, "cal") for row in grp]
        raw = [measured(row, "raw") for row in grp] if has_raw else None
        print(f"=== gt (x={gt_x:.0f}, y={gt_y:.0f}, z={gt_z:.0f}) mm | n={len(grp)} ===")
        for idx, (name, gt_val) in enumerate(zip(AXES, (gt_x, gt_y, gt_z))):
            if name in ("x", "y") and not has_gt_xy:
                continue
            cal_vals = [point[idx] for point in cal]
            raw_vals = [point[idx] for point in raw] if raw is not None else None
            print(format_axis(name, gt_val, cal_vals, raw_vals, gt_z))
        print()

    if args.plots:
        save_plots(args.csv_path, groups)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
