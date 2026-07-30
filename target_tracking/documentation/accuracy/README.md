# XYZ accuracy testing

Tools for measuring the spatial-coordinate (X, Y, Z) accuracy of the OAK-D pipeline against
ruler-measured ground truth. Requires a physical OAK-D connected over USB3.

## Capture

Run the pipeline with logging enabled, telling it the target's true position in mm:

```
python main_2025.py --log documentation/accuracy/accuracy_log_<date>.csv --gt-x 0 --gt-y 0 --gt-z 1000
```

In the window:
- A green box with live `X Y Z` is drawn on each detected person.
- Place ONE person at the measured position; everyone else stands behind the camera.
- Press `r` to start/stop recording (banner turns green `REC` only when exactly one person is
  in view — bystanders auto-pause it).
- Hold still ~5 s, then press `q` to quit. Repeat for each position, appending to the same CSV.

CSV columns: `timestamp, gt_x_mm, gt_y_mm, gt_z_mm, target_id, raw_x_mm, raw_y_mm, raw_z_mm,
cal_x_mm, cal_y_mm, cal_z_mm` (raw = straight from device, cal = after calibration).

## Analyze

```
python documentation/accuracy/analyze_accuracy.py documentation/accuracy/accuracy_log_<date>.csv
```

Reports per-position median error vs ground truth, in mm and as a percentage of range (the
ground-truth Z), with an IDEAL (<=5%) / PASS (<=10%) / FAIL verdict per axis. Sessions whose
spread is too large to be a still single subject are flagged `NOISY` (recapture them). Add
`--plots` to save a Z-over-time PNG per position (needs matplotlib).

## Files
- `accuracy_log_2026-06-10.csv` — clean capture set (this branch).
- `progress_report_2026-06-10.md` — results, calibration findings, and inaccuracy factors.
- `accuracy_log_2026-04-30.csv`, `accuracy_log_2026-05-07.csv`, `calibration.png` — prior
  baseline data/calibration from the depthai1 branch.
