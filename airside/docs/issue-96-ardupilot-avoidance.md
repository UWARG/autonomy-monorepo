# Issue 96 — Obstacle avoidance options, OAK-D, and ArduPilot SITL evidence

**Baseline, not a locked decision:** the completed demo uses **ArduPilot's built-in avoidance
stack** — the companion computer feeds the flight controller obstacle data over MAVLink and the
autopilot does the avoiding. This document proves that integration pattern end-to-end in SITL
(see `airside/scripts/avoidance/`), compares it with companion-planner and PX4 alternatives, and
defines the experiments needed without committing to a particular camera or airframe now.

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

## 6. Separate the two decisions: sensing and planning

The original recommendation coupled a 360° LiDAR to ArduPilot. They are actually independent:

1. A **sensor/coverage layer** decides which parts of the vehicle's motion envelope are observed.
2. An **avoidance layer** decides whether to stop, steer locally, or plan a new route.

That separation lets us test the OAK-D with the already-proven ArduPilot path now, then change
either the sensor or planner later without discarding the first experiment.

### 6.1 Sensor and coverage comparison

| Option | Useful coverage | Pros | Cons / failure cases | Integration and cost |
|---|---|---|---|---|
| **OAK-D / forward stereo depth** | Forward cone only. As examples, original OAK-D is 72° H × 49° V and OAK-D S2 is 80° H × 55° V; no model selection is required for this research. | Dense depth; 20–30 Hz is realistic; on-device stereo/ROI processing; ROS2 Humble support. The selected camera will always face the commanded motion direction. | Passive stereo degrades on textureless/repetitive, transparent, reflective, very thin, and poorly lit surfaces; sunlight can defeat Pro-model IR assistance. Coverage must include the vehicle width and expected sideslip, not only the path centreline. | Software bridge required. Keep its depth/sector input generic so later hardware can change. This branch's `camera/src/oakd.py` is empty, and the existing target-tracking accuracy report is a controlled person-ranging test—not generic obstacle recall. |
| **LightWare SF45/B** | Configurable horizontal scan up to 320° in one plane. | Native ArduPilot driver; good outdoor range; 59 g; almost no companion load; wide horizontal coverage. | One oscillating beam is not 3D coverage: it can miss overhangs, wires, obstacles above/below the scan plane, and geometry shifted by aircraft pitch/roll. Up to 5 sweeps/s. | Listed at **US$449 excluding tax** (checked July 2026), 5 V / 300 mA typical. Fastest hardware path, but should not be called low-cost. |
| **OAK-D + SF45/B** | Dense forward 3D cue plus wide horizontal plane. | Complementary failure modes; supports forward detail and lateral awareness; ArduPilot can accept multiple proximity sensors. | More mass, power, mounting, calibration, and fusion tests; still no true rear/up/down volume. | Medium effort and highest near-term cost. Add only if forward-depth reliability tests justify it. |
| **True 3D LiDAR (future benchmark)** | Example Livox Mid-360: 360° H, -7° to +52° V. | Real 3D points, long outdoor range, much better map/planner input. | Example is 265 g, 6.5 W, 200k points/s; needs host filtering, mapping, state estimation, and a custom planner. | High effort/cost; inappropriate as the first issue-96 experiment but useful if future comps demand general 3D navigation. |
| **Fences only** | Known, surveyed zones rather than sensed obstacles. | Highest determinism; zero perception; already part of competition safety infrastructure. | Cannot react to people, vehicles, or unmodeled structures. | Keep regardless of the live-sensor choice. |

Sources: [OAK-D hardware](https://docs.luxonis.com/hardware/products/OAK-D),
[OAK-D S2](https://docs.luxonis.com/hardware/products/OAK-D%20S2),
[Luxonis stereo limitations](https://docs.luxonis.com/hardware/platform/features/depth/),
[SF45/B product data](https://lightwarelidar.com/shop/sf45-b-50-m/), and
[Livox Mid-360 specifications](https://www.livoxtech.com/mid-360/specs).

### 6.2 What the OAK-D can offer

The OAK-D is a **depth/perception sensor, not an avoidance controller**. There are three useful
integration levels:

| OAK-D output | Avoidance owner | Benefit | Trade-off | Recommendation |
|---|---|---|---|---|
| Robust horizontal sectors → MAVLink `OBSTACLE_DISTANCE` | ArduPilot simple avoidance / BendyRuler | Reuses the proven demo and 72-sector interface; lowest effort; planner is sensor-agnostic. | Collapses 3D depth into a 2D boundary; BendyRuler remains a limited local planner. | **Build first.** |
| Selected 3D obstacle vectors → ArduPilot `OBSTACLE_DISTANCE_3D` | ArduPilot 3D obstacle database / BendyRuler | Preserves obstacle height and direction. | One vector per MAVLink message; point cloud must be clustered/subsampled; ArduPilot's vertical BendyRuler is not a general 3D trajectory optimizer. | Evaluate only after sectors work. |
| Depth/point cloud → ROS2 map/local planner → flight setpoints | Companion computer | Highest flexibility and true 3D planning potential. | Requires odometry, mapping, trajectory generation, compute budgeting, command watchdogs, and substantially more safety-critical WARG code. | Future path if FC planning fails the “stay on task” scenarios. |

Do **not** use the minimum pixel of each image column as the flight distance. A single stereo
speckle, propeller edge, or ground pixel would create false stops, while invalid depth could look
clear. The first bridge should:

- rectify and confidence-filter depth; enable left-right consistency checking;
- mask propellers/landing gear and either remove the ground plane or restrict the vertical ROI;
- divide the image into angular sectors using calibration, not pixel count alone;
- choose the nearest spatially-supported cluster or a conservative low percentile, not one pixel;
- require temporal persistence while still admitting genuinely sudden obstacles;
- encode invalid/unknown separately from clear and publish timestamps/sequence health;
- expire stale frames and independently command BRAKE/LOITER if the sensor or publisher dies.

Luxonis reports ideal-target error below 2% under 4 m and below 4% from 4–7 m for a normal-FOV
75 mm-baseline OAK at 800P. Those are useful calibration expectations, **not a safety guarantee**:
they use a well-textured target and do not measure obstacle false negatives. See
[depth accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/),
[stereo configuration](https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth/),
[SpatialLocationCalculator](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/spatial_location_calculator),
and [DepthAI ROS](https://docs.luxonis.com/software-v3/depthai/ros/).

### 6.3 Avoidance-controller comparison

| Controller | Live obstacle | Stop safely | Stay on task / detour | Main advantages | Main disadvantages | Relative effort |
|---|---:|---:|---:|---|---|---:|
| **Fences + ArduPilot Dijkstra** | No | Yes, for known zones | Yes | Most deterministic; no live perception dependency. | Only predeclared geometry. | Low |
| **ArduPilot simple avoidance** | Yes | Yes | No | Already demonstrated; minimal companion code; protects pilot-style velocity commands on tested firmware. | Stops/slides only; margin undershoot; feed fails open; current docs do not promise every GUIDED command path. | Low |
| **ArduPilot BendyRuler** | Yes | Usually | Yes, locally | Already demonstrated in AUTO and WPNav-routed GUIDED; keeps current FC/competition architecture. | Local minima and narrow passages; about 1 Hz in the tested setup; horizontal/vertical modes are limited rather than full 3D planning; feed fails open. | Low–medium |
| **Small companion reactive planner** | Yes | WARG-owned | Locally | Autopilot-agnostic and can be tailored to target following; more control than stop-only behavior. | WARG owns safety-critical steering, local minima, latency, setpoint, and watchdog logic. | Medium–high |
| **Full companion 3D mapper/planner** | Yes | WARG-owned | Best potential | Exploits dense OAK/3D LiDAR; can plan trajectories in complex unknown spaces. | Needs robust odometry/map/compute; largest validation surface. Academic options are not drop-in competition components. | High |
| **PX4 Collision Prevention** | Yes | Yes | Minor steering only | Conservative no-data default: motion into unknown sectors is blocked; data loss stops XY and then enters HOLD. | Acceleration-based Position mode only; not mission route-around; switching FCs adds requalification. | High migration cost |
| **PX4 custom ROS2 mode/planner** | Yes | WARG-owned | Potentially | Deep ROS2 integration and replaceable flight modes. | The maintained interface is experimental; WARG still supplies the planner and must migrate/requalify the FC stack. | Very high |

### 6.4 Alternatives to ArduPilot: conclusion

**Do not switch to PX4 for issue 96.** PX4 Collision Prevention has a better fail-closed default
than ArduPilot: unknown sectors block movement, >0.5 s total data loss blocks XY motion, and >5 s
switches to HOLD. However it is a Position-mode collision limiter, not maintained mission
avoidance. PX4 removed its Path Planning Interface in v1.15 and explicitly says the old Mission
Obstacle Avoidance/Safe Landing path is unsupported; the associated `PX4-Avoidance` ROS1 project
was archived in 2024. The replacement ROS2 flight-mode interface is experimental. See
[PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention),
[removed Path Planning Interface](https://docs.px4.io/main/en/computer_vision/path_planning_interface),
and [archived PX4-Avoidance](https://github.com/PX4/PX4-Avoidance).

Companion-side frameworks/planners are useful references, not near-term dependencies:
[Aerostack2](https://aerostack2.github.io/) is a full ROS2 aerial robotics framework whose
adoption would compete with the existing airside architecture; [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
is an academic high-performance planner whose primary integration is ROS1/catkin. The old
[UWARG obstacle-avoidance repository](https://github.com/UWARG/obstacle-avoidance) has only a
minimal public README and no releases, so reuse should be decided only after a code/flight-log
audit rather than assumed from its existence.

### 6.5 Prioritization recommendation

1. **Keep fences as the deterministic baseline.** They solve known competition/site hazards.
2. **Build a forward-depth → robust `OBSTACLE_DISTANCE` bridge first, using OAK-D as the reference
   implementation.** Keep sensor acquisition behind a generic interface so the eventual camera
   can change. This directly reuses the SITL seam. Pair it with ArduPilot simple avoidance and
   BendyRuler on the exact firmware and command paths intended for flight.
3. **Make fail-closed behavior part of that prototype, not a later enhancement.** On stale or
   missing depth/output, the companion must request BRAKE/LOITER, confirm the command, and alert.
4. **Exploit the confirmed camera-forward motion constraint.** Forward depth can be the sole live
   sensor for commanded translation, so defer the SF45/B. Still include full airframe width,
   braking sideslip, wind disturbance, and yaw transients in the tested collision envelope. Add
   wider sensing only if forward-depth tests show an actual reliability gap.
5. **Escalate to a companion 3D planner only if BendyRuler fails required detour scenarios.**
   Do not pay the mapping/planning complexity before demonstrating that need.

## 7. Integration architectures — where the bridge lives

The FC-side configuration is identical in every option (§2's `PRX1_TYPE` / `AVOID_*` /
`OA_*` params); what differs is **which process produces the `OBSTACLE_DISTANCE` stream
and how it reaches the FC**. Four shapes, not mutually exclusive:

| Option | Shape | Companion software | Fits |
|---|---|---|---|
| **A. FC-direct sensor** | SF45/B (or similar) wired straight to an FC serial port, `PRX1_TYPE=8` | none | fastest to flight-ready once the sensor exists |
| **B. Standalone pymavlink daemon** | Python process: sensor driver → 72 sector distances → `OBSTACLE_DISTANCE` at 10–15 Hz over a dedicated FC telem port | one script — exactly the demo's shape (swap `wall_sector_distances()` for a real driver) | the comp repo's plain-pymavlink stack; this is the officially documented pattern (RealSense guide) |
| **C. ROS2 / MAVROS plugin** | a ROS2 node publishes `sensor_msgs/LaserScan` on `/mavros/obstacle/send`; the `obstacle_distance` plugin in mavros_extras converts and sends it | one small node; the plugin ships with mavros and loads by default under the APM plugin config | the airside engine (already MAVROS-based) |
| **D. Fences only** | pre-declared exclusion fences; Dijkstra or fence-stop | none | pre-surveyed obstacles; zero perception |

B and C emit the identical MAVLink message, so bridge logic written for one moves to the
other by swapping the transport; either can later be retired to A if a native LiDAR lands.

**Gotchas on the MAVROS route (C), verified in the plugin/firmware source:**

- The plugin assumes the incoming LaserScan is already **body-FRD with clockwise-increasing
  angles** — it does *not* convert from ROS's usual CCW (REP-103) convention; a compliant
  scan comes out mirrored. Publish FRD/clockwise or correct with `PRX1_YAW_CORR`/`PRX1_ORIENT`.
- ArduPilot **ignores `OBSTACLE_DISTANCE`'s `frame` field** entirely (sectors are always
  interpreted body-frame, offset by `PRX1_YAW_CORR + angle_offset`). Still set the plugin's
  `mav_frame: BODY_FRD` for spec correctness.
- The link must be MAVLink 2 (`SERIALx_PROTOCOL=2`) for the 72-sector float-increment
  extensions; scans with more than 72 rays are downsampled by per-bin minimum.

### 7.1 Link topology on the real aircraft

The officially documented pattern is a **dedicated FC telemetry port for the companion at
921600 baud** (`SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921`), with the bridge talking directly
to that serial port — no router daemon involved. The GCS radio lives on a *different* FC
port, and ArduPilot natively routes MAVLink between its ports, so both coexist. Bandwidth
is a non-issue on that link (~179 bytes/message → 10 Hz ≈ 1.9 % of 921600 baud) but would
consume roughly a third of a 57600 radio link — set `SERIALx_OPTIONS` bit 10 ("don't
forward MAVLink") on the slow GCS port so the obstacle stream is not forwarded to it.
A companion-side router (mavlink-router / MAVProxy / mavp2p) is only needed when several
companion processes must share one UART (bridge + MAVROS + logging); the ArduPilot
Raspberry Pi guide documents all three and prefers `mavlink-routerd` when the Pi is loaded.

### 7.2 Failure behavior: ArduPilot fails *open* when the feed dies

Verified in Copter-4.5 source — a dead obstacle feed is treated as "no obstacles", not as
an emergency:

| Feed dies at t=0 | What happens |
|---|---|
| +0.5 s | proximity status → NoData (`PROXIMITY_MAV_TIMEOUT_MS`); SYS_STATUS proximity health bit clears (Mission Planner shows "Bad Proximity"); **no STATUSTEXT** |
| +1–1.75 s | proximity boundary faces expire → **simple avoidance silently stops limiting velocity** |
| +10 s (`OA_DB_EXPIRE`) | obstacle database empties → **BendyRuler plans straight as if the world were empty** (until then it avoids stale "ghost" points) |
| never | no failsafe, no mode change — the only hard gate is the *prearm* check ("PRX1: No Data"), which never re-runs in flight |

Implication: if the bridge process crashes mid-flight, avoidance evaporates within seconds
with only a GCS health bit to show for it. **Fail-closed behavior is a companion-side
responsibility** — the bridge (or a sibling watchdog) should monitor its own output and
command BRAKE/LOITER on failure, mirroring the deadman pattern the follow stack already
uses. This belongs in the reliability gates (§8).

## 8. Proposed experiments and reliability gates

### Phase 0 — freeze the test contract

- Treat the camera-forward motion rule and a sensor-agnostic forward-depth/sector interface as the
  research contract. Once prototype hardware is selected, record its lens/FOV and mount transform
  together with the target aircraft, ArduCopter version/hash, avoidance parameters, maximum
  airspeed, and command path before qualification.
- Separate a **required safety gate** (“does not contact an obstacle”) from an optional mission
  gate (“detours and reaches the original goal”). A safe stop can pass the first and fail the second.
- Derive the speed cap from measured behavior, not sensor maximum range:
  `required_range = speed × end_to_end_latency + speed²/(2 × braking_acceleration)
  + state/sensing uncertainty + reserve`.

### Phase 1 — replay and bench

- Build recorded-depth replay before live flight-controller output so algorithms are deterministic
  and regressions can run without the camera.
- Test textured wall, low-texture tarp, foliage, thin pole/branch/netting, reflective and transparent
  surfaces if mission-relevant, direct sun/backlight, motion blur, roll/pitch, and a moving obstacle.
- Report per-class detection recall/false-clear rate, false-stop rate, valid-depth fill, reliable range,
  sector jitter, and p50/p95/p99 camera-to-MAVLink latency. XYZ accuracy alone is insufficient.
- Fault-inject USB disconnect, frozen frames, process death/restart, CPU saturation, and MAVLink loss.

### Phase 2 — SITL/closed-loop qualification

- Run static wall, pop-up obstacle, crossing/moving obstacle, narrow passage, U-shape/local minimum,
  obstacle during follow, feed dropout, planner restart, and changing vehicle yaw.
- Sweep speeds, margins, frame rates, latency, and packet loss; use many randomized seeds rather
  than one hand-picked run. Pin every firmware/config artifact in the log.
- Acceptance: zero collisions/clearance breaches in the agreed suite; stale data is never interpreted
  as clear; feed kill causes acknowledged BRAKE/LOITER within the agreed deadline; RC override and
  avoidance disable remain available.

### Phase 3 — staged physical flight

- Stationary/tethered or prop-off data collection → soft obstacle at low speed → representative
  speed and lighting → combined competition behavior. Keep a pilot/kill path and geofence active.
- Increase speed only when measured reliable range exceeds the stopping-distance budget with reserve.
- Decide whether any wider-coverage sensor is needed only after the forward-depth failure data.

### Integration hygiene

- Retest the Copter-4.5 observations on the exact deployed firmware. Current ArduPilot docs describe
  simple avoidance in AltHold/Loiter and BendyRuler in AUTO/GUIDED; the observed GUIDED-velocity
  protection is valuable evidence but should not be treated as a cross-version API guarantee.
- Any GUIDED position target requiring path planning needs `GUID_OPTIONS=64` or an AUTO mission.
- Tune `AVOID_MARGIN` / `OA_MARGIN_MAX` at flight speed and verify simultaneous sensor fusion if a
  second proximity sensor is added.

## 9. References

- ArduPilot object avoidance: <https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html>
- BendyRuler: <https://ardupilot.org/copter/docs/common-oa-bendyruler.html> · Dijkstra: <https://ardupilot.org/copter/docs/common-oa-dijkstras.html>
- Simple avoidance: <https://ardupilot.org/copter/docs/common-simple-object-avoidance.html>
- Companion depth-camera pattern (RealSense): <https://ardupilot.org/copter/docs/common-realsense-depth-camera.html>
- SF45/B setup: <https://ardupilot.org/copter/docs/common-lightware-sf45b.html>
- SF45/B manufacturer data: <https://lightwarelidar.com/shop/sf45-b-50-m/>
- `OBSTACLE_DISTANCE` message: <https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE>
- `OBSTACLE_DISTANCE_3D` message: <https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D>
- Luxonis OAK-D and depth: <https://docs.luxonis.com/hardware/products/OAK-D> · <https://docs.luxonis.com/hardware/platform/features/depth/> · <https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/>
- PX4 comparison: <https://docs.px4.io/main/en/computer_vision/collision_prevention> · <https://docs.px4.io/main/en/computer_vision/path_planning_interface>
- MAVROS `obstacle_distance` plugin (ROS2): <https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/obstacle_distance.cpp>
- MAVLink routing between FC ports: <https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html> · companion/router options: <https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html>
- Proximity feed timeout + fail-open behavior: `libraries/AP_Proximity/AP_Proximity_MAV.cpp` (`PROXIMITY_MAV_TIMEOUT_MS`) and `libraries/AC_Avoidance/AP_OADatabase.cpp` (`OA_DB_EXPIRE`) on <https://github.com/ArduPilot/ardupilot/tree/Copter-4.5>
- Prior WARG implementation: <https://github.com/UWARG/obstacle-avoidance> and the CV-space Confluence page above
- Demo + logs: `airside/scripts/avoidance/` in this repo
