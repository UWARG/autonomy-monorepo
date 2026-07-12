# Issue 96 — ArduPilot obstacle-avoidance integration demo

Proves the companion-computer integration pattern for ArduPilot-native
obstacle avoidance in SITL, with **no hardware**: a synthetic wall is streamed
to the flight controller as MAVLink `OBSTACLE_DISTANCE` (72 × 5° sectors,
10 Hz, body frame) — exactly what a real depth-camera or LiDAR bridge would
send — and ArduPilot's proximity + BendyRuler stack does the avoiding.

See `airside/docs/issue-96-ardupilot-avoidance.md` for the research write-up.

## Files

- `avoidance_demo.py` — the bridge + scenario runner (runs inside the SITL
  container; only dependency is `pymavlink`, already in the image).
- `sitl_avoidance.parm` — the avoidance parameters (`PRX1_TYPE=2`,
  `AVOID_ENABLE=7`, `OA_TYPE=1` BendyRuler, `OA_MARGIN_MAX=3`). These are
  reboot-required, so they load as boot defaults via `--defaults`.
- `Dockerfile.sitl` — headless ArduCopter 4.5 SITL image (`warg/sitl:latest`);
  identical to the shared image from the issue-24 work, duplicated so this
  demo is self-contained.

## Run

```bash
# 1. Build the image once (skip if warg/sitl:latest already exists):
docker build -f Dockerfile.sitl -t warg/sitl:latest .

# 2. Fresh SITL with the avoidance parameters, demo dir mounted at /demo:
docker rm -f sitl-96 2>/dev/null
docker run -d --name sitl-96 -v "$PWD":/demo warg/sitl:latest bash -lc \
  'cd /ardupilot && exec build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
   --sim-address=127.0.0.1 \
   --defaults Tools/autotest/default_params/copter.parm,/demo/sitl_avoidance.parm'

# 3. Run one scenario (fresh container boot per scenario for clean state):
docker exec sitl-96 python3 /demo/avoidance_demo.py --scenario wall_auto

# ...or the whole suite (fresh boot per scenario, summaries collected):
./run_all.sh
```

## Scenarios and expected outcomes

| Scenario | Command path | Expected |
|---|---|---|
| `clear_guided` | GUIDED goto 40 m north, no obstacle | PASS — goal reached (baseline) |
| `wall_guided` | GUIDED goto through the wall (plain `SET_POSITION_TARGET`) | **FAIL — documents the gap**: position targets bypass BendyRuler |
| `wall_guided_wpnav` | same + `GUID_OPTIONS=64` (targets routed via WPNav) | PASS — detours around, stays on task |
| `wall_guided_vel` | GUIDED velocity streaming at 2 m/s (follow-stack pattern) | PASS — simple avoidance stops it before the wall (no detour) |
| `wall_auto` | AUTO waypoint mission through the wall | PASS — BendyRuler detours, mission continues |

The wall is a 12 m-wide segment 20 m north of home; BendyRuler margin
(`OA_MARGIN_MAX`) is 3 m. A wall scenario FAILs only on an actual wall
crossing ("breached"); minimum clearance is reported separately
(`clearance_ok`, 1 m floor).

Telemetry for each run is written to `logs/<scenario>.jsonl`
(`north/east/alt`, true distance-to-wall, elapsed time); the script prints a
JSON summary + PASS/FAIL verdict and exits nonzero on FAIL.
See `airside/docs/issue-96-ardupilot-avoidance.md` for what these results
mean for the real airframe.
