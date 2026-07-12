# Issue 96 — Obstacle avoidance with ArduPilot: research, integration path, and SITL evidence

**Scope decision:** obstacle avoidance is done **with ArduPilot's built-in avoidance stack** —
the companion computer feeds the flight controller obstacle data over MAVLink and the autopilot
does the avoiding. This document explains what that means, proves the integration pattern
end-to-end in SITL (see `airside/scripts/avoidance/`), and lays out the path to the real airframe.

---

## 1. ArduPilot in one page (onboarding)

ArduPilot is the open-source **autopilot firmware running on the drone's flight controller**
(FC — a Pixhawk/Cube-class board). It is not a library our Python imports; it is the
always-running program that actually flies the aircraft:

- attitude stabilization and motor mixing,
- position/velocity control and the flight modes (STABILIZE, LOITER, **GUIDED**, **AUTO**, RTL, BRAKE, …),
- state estimation (EKF3 fusing IMU/GPS/baro/compass),
- failsafes, geofences, arming checks.

The multirotor build is **ArduCopter**. Our stack already talks to it everywhere:

```
companion computer (Pi)                         flight controller
┌─────────────────────────────┐    MAVLink     ┌──────────────────┐
│ our Python / ROS2 code       │ ⟵──────────⟶ │ ArduPilot         │
│ (pymavlink, MAVROS, engine)  │  serial/TCP    │ (ArduCopter)      │
└─────────────────────────────┘                └──────────────────┘
```

Every `SET_POSITION_TARGET`, mode change, or heartbeat that `mav_comms` or the airside engine
sends is a MAVLink message to ArduPilot. **SITL** (software-in-the-loop) is the same firmware
compiled for the desktop with simulated physics — which is why everything in this document was
provable without hardware.

## 2. ArduPilot's obstacle-avoidance subsystem

Three layers, all configured by parameters — no FC code changes:

### 2.1 Getting obstacle data in (the *proximity* layer, `PRX_`)

| Route | How | Companion work |
|---|---|---|
| Natively-supported 360° scanning LiDAR (e.g. LightWare SF45/B) | wired to the FC, `PRX1_TYPE=8` | none |
| **Companion-fed distances (what we demo)** | stream MAVLink **`OBSTACLE_DISTANCE`** (up to 72 × 5° sectors) or `DISTANCE_SENSOR` at ~10 Hz, `PRX1_TYPE=2` | a bridge: sensor → sector distances → MAVLink |

`OBSTACLE_DISTANCE` fields: 72 uint16 distances (cm), sector increment (deg), min/max range,
frame (`MAV_FRAME_BODY_FRD` = sector 0 straight ahead). Values > max-range mean "clear".
This is the same pattern as the official Intel RealSense integration, and it is
sensor-agnostic: anything that can produce "distance per direction" can feed it.

### 2.2 Avoidance behaviors

| Behavior | Params | Acts in | What it does |
|---|---|---|---|
| **Simple avoidance** (`AC_Avoid`) | `AVOID_ENABLE`, `AVOID_MARGIN`, `AVOID_BEHAVE` | pilot modes (Loiter/AltHold) **and GUIDED velocity control** (verified, §4) | limits/zeroes velocity so the vehicle stops (or slides) before the obstacle |
| **BendyRuler path planning** | `OA_TYPE=1`, `OA_MARGIN_MAX`, `OA_BR_*` | AUTO, GUIDED (see §4 caveat), RTL | probes candidate headings around the sensed obstacle and steers around it, then resumes the route — "stay on task" |
| **Dijkstra path planning** | `OA_TYPE=2` | AUTO, GUIDED, RTL | optimal path around **pre-declared fences/stay-out zones only** — blind to live sensor data |

Docs: [object avoidance landing page](https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html),
[BendyRuler](https://ardupilot.org/copter/docs/common-oa-bendyruler.html),
[Dijkstra](https://ardupilot.org/copter/docs/common-oa-dijkstras.html),
[simple avoidance](https://ardupilot.org/copter/docs/common-simple-object-avoidance.html),
[RealSense companion pattern](https://ardupilot.org/copter/docs/common-realsense-depth-camera.html).

## 3. What we demonstrated in SITL

Setup (`airside/scripts/avoidance/`): headless ArduCopter 4.5 SITL in Docker; a Python
"bridge" (`avoidance_demo.py`) ray-casts a synthetic 12 m-wide wall 20 m north of home into
72 body-frame sectors and streams it as `OBSTACLE_DISTANCE` at 10 Hz — byte-for-byte what a
real depth-camera/LiDAR bridge would send. Parameters: `PRX1_TYPE=2`, `AVOID_ENABLE=7`,
`OA_TYPE=1` (BendyRuler), `OA_MARGIN_MAX=3`. The vehicle is then commanded through the wall
five different ways. One fresh SITL boot per scenario; telemetry logged to JSONL.

| # | Scenario | Command path | Result |
|---|---|---|---|
| 1 | `clear_guided` | GUIDED goto 40 m north, no obstacle | goal in 8.2 s (baseline sanity) |
| 2 | `wall_guided` | GUIDED goto through the wall (plain `SET_POSITION_TARGET`) | **flew straight through the wall** (min dist 0.38 m, breached) — position targets bypass OA |
| 3 | `wall_guided_wpnav` | same + `GUID_OPTIONS=64` (route position targets via WPNav) | **avoided**: kept 3.84 m (margin is 3 m), detoured around, goal in 14.3 s |
| 4 | `wall_guided_vel` | GUIDED **velocity streaming** at 2 m/s toward the wall (the follow-stack pattern) | **stopped before the wall** (no crossing; came to 0.77 m with default `AVOID_MARGIN=2`) |
| 5 | `wall_auto` | AUTO waypoint mission through the wall | **avoided**: kept 3.79 m, detoured, waypoint reached in 23 s |

*(Numbers from the recorded suite run; reproduce with `./run_all.sh` — summaries land in
`logs/summaries.txt`.)*

Feed ingestion was independently confirmed in every run: once `OBSTACLE_DISTANCE` flows, the
FC echoes `DISTANCE_SENSOR` proximity telemetry back (hundreds of messages per run) and the
`PRX1` prearm check passes.

## 4. Findings that matter for integration

1. **The integration pattern works, end to end.** A ~40-line sender loop is the entire
   companion-side obligation: 72 sector distances, 10 Hz, body frame. Swapping the synthetic
   wall for a real sensor driver is the only change (`wall_sector_distances()` is the seam).
2. **Plain GUIDED position targets bypass path-planning avoidance** (scenario 2 — empirically
   confirmed, not just forum lore). The fix is `GUID_OPTIONS=64` so guided position targets go
   through WPNav, which carries the OA wrapper (scenario 3). Any comp code that sends GUIDED
   gotos **must set this parameter or fly missions in AUTO**.
3. **GUIDED velocity streaming *is* protected by simple avoidance in Copter 4.5** — the
   vehicle stopped rather than hitting the wall (scenario 4). Two qualifications:
   it *stops*, it does not steer around (no "stay on task"); and at 2 m/s it undershot
   `AVOID_MARGIN=2` to 0.77 m — margins must be tuned and validated at the speeds we
   actually fly. This is directly relevant to the drone-follow stack (issue #24), which
   streams GUIDED velocity setpoints.
4. **Dijkstra is fences-only.** If comp obstacles are surveyable in advance, declaring them as
   stay-out fences gives reliable avoidance with **zero perception work** — worth remembering
   as the lowest-effort competition option. (The 2026 CONOPS defines hard/soft geofence
   boundaries anyway; fence infrastructure will exist.)
5. **Speedup note:** the demo runs SITL in real time; ArduPilot's OA planner runs at ~1 Hz,
   so approach speeds and margins interact — another reason reliability gates must test at
   mission speeds.

## 5. Competition context

Verified against the 2026 AEAC CONOPS v1.2 (43 pages,
[PDF](https://www.aerialevolution.ca/wp-content/uploads/2025/12/2026-AEAC-CONOPS-v1.2-2025-11-28.pdf)):
there is **no scored physical obstacle-avoidance requirement** in 2026. Tasks are Fire
Reconnaissance and Fire Extinguishing at Area X.O (Ottawa); the only "avoidance" language is
ATC-commanded *traffic* deconfliction ("Yield to Medevac"), plus soft/hard flight boundaries
(kill required beyond the hard boundary). So issue 96 is a **defensive-reliability capability**
— protecting the aircraft around site structures and de-risking future comps — exactly
matching the issue's "may be part of current/future comps depending on reliability".

## 6. Sensor options for the real airframe

The bridge is sensor-agnostic; the choice is about coverage, weight, and effort:

| Option | Coverage | Companion work | Notes |
|---|---|---|---|
| **LightWare SF45/B scanning LiDAR** (~59 g) | ~320° horizontal | **none** (native `PRX1_TYPE=8` FC driver) | WARG has prior art: the old [obstacle-avoidance repo](https://github.com/UWARG/obstacle-avoidance) + [Confluence page](https://uwarg-docs.atlassian.net/wiki/spaces/CV/pages/2599288845/Obstacle+Avoidance) were built around this sensor. Recommended primary. |
| **OAK-D stereo depth** (already flown) | forward ~70–120° | depth frame → column-minimum sector distances → `OBSTACLE_DISTANCE` (the demo's seam) | software-only; guards only the direction of view; good complement, weak as sole sensor for lateral/backward motion |
| **Fences only** | n/a | none | for pre-surveyed obstacles; zero perception; pairs with Dijkstra or plain fence stop |

Recommendation: request an SF45/B (or similar native proximity LiDAR) as the primary obstacle
sensor; build the OAK-D bridge as the software path where camera coverage suffices — both
converge on the identical FC configuration proven here.

## 7. Proposed next steps

1. **Review this doc + demo** with the lead / Sophie; confirm sensor request (SF45/B).
2. **OAK-D → `OBSTACLE_DISTANCE` bridge** (software ticket, no new hardware): replace
   `wall_sector_distances()` with depth-image sector minima; bench-test pointing the camera at
   real objects. Natural split candidate with the co-assignee.
3. **Reliability gates** (per the issue's condition): scripted SITL scenario suite —
   pop-up obstacle, obstacle during follow, sensor dropout, approach at mission speeds —
   with an N/N pass bar, then a physical demonstration with soft obstacles at a flight test
   (M-series) before any comp use.
4. **Parameter hygiene for other tickets**: any GUIDED-goto code needs `GUID_OPTIONS=64`;
   margin params (`AVOID_MARGIN`, `OA_MARGIN_MAX`) tuned to flight speeds.

## 8. References

- ArduPilot object avoidance: <https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html>
- BendyRuler: <https://ardupilot.org/copter/docs/common-oa-bendyruler.html> · Dijkstra: <https://ardupilot.org/copter/docs/common-oa-dijkstras.html>
- Simple avoidance: <https://ardupilot.org/copter/docs/common-simple-object-avoidance.html>
- Companion depth-camera pattern (RealSense): <https://ardupilot.org/copter/docs/common-realsense-depth-camera.html>
- SF45/B setup: <https://ardupilot.org/copter/docs/common-lightware-sf45b.html>
- `OBSTACLE_DISTANCE` message: <https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE>
- Prior WARG implementation: <https://github.com/UWARG/obstacle-avoidance> and the CV-space Confluence page above
- Demo + logs: `airside/scripts/avoidance/` in this repo
