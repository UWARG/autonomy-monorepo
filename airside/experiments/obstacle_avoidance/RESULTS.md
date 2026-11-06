# Moving traffic flight-test results

Status on 2026-11-06: **PR #237 is ready for review, but live obstacle avoidance is not ready for a props-on flight test.** This branch is based on #232 at `7c521c9`.

## Verified

- `gnc` and replay tests: 149 passed. Rebuilt Airside image: 70 `colcon test` checks passed.
- Mock SITL: moving crossing 3/3, plus clear, multi-aircraft, turn, data dropout, blocked route, and pilot takeover passed. In the moving, multi-aircraft, and turn runs, the FCU speed setting was 1.5 m/s, measured peak was at most 1.527 m/s, and the smallest raw keep-away margin was 13.963 m. Each run had one MAVROS setpoint publisher and saved a rosbag.
- Generated-track SITL replay: 36 snapshots at about 1 Hz, goal reached, measured peak 1.513 m/s, and raw keep-away margin 11.959 m.
- The separate #181 static-obstacle regression passed 3/3; its full readiness check passed 5/5.
- Receive-only AEAC capture after the server scenario changed: 60 valid snapshots at 0.953–1.031 s intervals, with no parser errors and no telemetry sent. Every snapshot contained WebSocket indices 0, 1, and 2. Their coordinates matched the server screenshots:

  | WebSocket index | Traffic track | Altitude | Speed |
  | --- | --- | ---: | ---: |
  | 0 | Three-point triangle | 50 m AGL | 5 m/s |
  | 1 | Two-point moving track | 10 m AGL | 5 m/s |
  | 2 | Stationary hold | 10 m AGL | 0 m/s |

  All three reported 5 m horizontal and 30 m vertical keep-away. The 50 m triangle is a server traffic track; the checked-in Airside route is a different 10 m triangle. The route matcher placed moving index 1 within 0.188 m of Airside route segment 2.

## Real-track SITL: not passed

One diagnostic replay kept all three traffic aircraft by using test-only own index 255. The controller held, requested pilot takeover, and did not reach the route-leg goal. Its smallest measured margin outside the raw keep-away was 6.816 m. At replay start, the route-leg origin was 12.85 m from moving index 1, inside its 14.75 m expanded avoidance radius. The finite capture later became stale, so the controller continued to hold.

The replay saved a rosbag and summary in the ignored `artifacts/replay-WMfRPwH5/` directory. Test-only index 255 does not establish our real aircraft index. This result is a failed goal-completion test despite staying outside the raw keep-away.

## Before live props-on flight

1. Resolve the real-track route blockage without reducing the raw keep-away. Pass three independent timing-preserving SITL replays with goal completion, separation, speed, data freshness, and one setpoint publisher checked.
2. Confirm our aircraft index. Run the authorized props-off check for both WebSocket connections, WGS84/AGL fields, approximately 1 Hz send and receive rates, and end-to-end latency.
3. Keep the first physical flight at 1 m/s. The 1.5 m/s result is SITL-only.

Earlier captures from the previous server scenario did not match this route. Screenshot-based mock tests used provisional keep-away values; the live capture above supersedes those assumptions. An experimental buffer reduction that breached keep-away was reverted.
