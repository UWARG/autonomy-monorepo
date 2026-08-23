# Obstacle Avoidance — Options, OAK-D, and Recommendation

> Paste-ready draft for a **new** Confluence page (CV space). Written for leads deciding
> whether this goes into comp. The runnable detail lives in the repo doc linked at the
> bottom; this page stays at the decision level.
>
> **Owner:** Junzhang Luo (+ Sam) · **Issue:** #96 · **Status:** Research + SITL demo done, decisions pending · July 2026

---

## Summary

- **Recommendation:** first build a **sensor-agnostic forward-depth → robust
  `OBSTACLE_DISTANCE` bridge**, using OAK-D as the reference implementation, and use the ArduPilot
  path already proven in SITL. Keep fences as the deterministic baseline.
- **Separate sensor from planner:** OAK-D, SF45/B, or another sensor can feed ArduPilot or a
  companion planner. ArduPilot simple avoidance stops; BendyRuler attempts a local detour.
- **OAK-D is promising, not turnkey:** it provides dense forward depth—not avoidance. It needs
  filtering, ground/airframe masking, robust sector extraction, stale-data handling, and outdoor
  obstacle testing. This branch has no ready OAK obstacle bridge.
- **PX4 is not a shortcut:** its Position-mode limiter fails closed on missing data, but PX4
  removed maintained Mission Obstacle Avoidance. Switching would add flight-controller migration
  while WARG would still need to build and validate the route planner.
- **Not required for AEAC 2026:** treat this as site-safety/future-comp work and require explicit
  reliability gates before competition use.

## Confirmed constraints and remaining decisions

1. **Camera-forward motion is guaranteed.** Forward-only depth is therefore a viable primary
   sensing architecture for commanded translation; tests must still cover the airframe width,
   braking sideslip, wind disturbance, and yaw transients.
2. **Hardware remains flexible.** The research does not choose an OAK model, mount, or airframe.
   Keep acquisition behind a generic depth/sector interface and freeze those details only when a
   prototype enters qualification.
3. **Remaining lead decisions:** endorse the staged reliability gates and priority. Defer SF45/B
   or other wide-coverage hardware unless forward-depth tests reveal a concrete gap.

---

## Why it matters

The 2026 AEAC mission (fire recon + extinguishing, Area X.O) has **no scored obstacle-avoidance
task** — only ATC traffic deconfliction and hard/soft geofences. So the value here is protecting
the aircraft around site structures and de-risking future competitions, not 2026 points.

---

## Differences and pros/cons

The sensor and controller are separate choices: coverage determines where the aircraft can safely
move; the controller determines whether it stops or continues around the obstacle.

### Sensor choices

| Option | Pros | Cons | When to choose it |
|---|---|---|---|
| **OAK-D / forward stereo** | Dense forward depth; 20–30 Hz is realistic; on-device stereo/ROI processing; ROS2 support; matches the camera-forward motion rule. | Passive stereo is weak on textureless/repetitive, reflective/transparent, thin, poorly lit, or motion-blurred objects; still needs a bridge and watchdog. | **Reference implementation.** Keep the interface generic so hardware can change. |
| **SF45/B** | 59 g; native ArduPilot integration; outdoor LiDAR; up to 320° horizontal. | A single scan plane, not 3D; up to 5 sweeps/s; can miss objects above/below the plane; listed at **US$449 before tax**, not “low cost.” | Add if lateral/rear coverage is mandatory or OAK-only reliability is inadequate. |
| **OAK-D + SF45/B** | Forward 3D detail plus wide horizontal awareness; complementary weaknesses. | More weight, power, mounting, and fusion tests; still incomplete vertical/rear volume. | Use only if testing shows both are necessary. |
| **True 3D LiDAR** | Best general 3D map/planner input and outdoor range. | Much heavier and more power/compute; custom filtering, mapping, and planning. | Future competition requirement, not the first issue-96 trial. |
| **Fences** | Deterministic and no perception dependency. | Known static hazards only. | Keep in every architecture. |

The original OAK-D has about 72° horizontal × 49° vertical FOV; the S2 is about 80° × 55°.
These illustrate the available envelope, not a model choice. Luxonis's ideal textured-target
accuracy numbers are calibration expectations, not proof of obstacle recall outdoors.

### Avoidance choices

| Option | Live obstacle? | Stays on task? | Pros | Cons / cost | Priority |
|---|---:|---:|---|---|---:|
| **ArduPilot simple avoidance** | Yes | No | Lowest effort; stopped in Copter 4.5 SITL. | Stops/slides; tested feed fails open; margin needs speed testing. | 1 for safety stop |
| **ArduPilot BendyRuler** | Yes | Usually, local detour | Proven in AUTO and WPNav-routed GUIDED on Copter 4.5; fits the current architecture. | Local minima/narrow passages; limited 3D planning; tested feed fails open. | **1** |
| **Companion reactive planner** | Yes | Locally | Tailored behavior; autopilot-agnostic. | WARG owns safety-critical steering, latency, setpoints, and watchdogs. | 2 if BendyRuler fails |
| **Companion 3D map/planner** | Yes | Best potential | Exploits dense depth; handles complex unknown spaces. | Highest compute, odometry, mapping, integration, and validation cost. | 3 / future |
| **PX4 Collision Prevention** | Yes | Minor steering only | Missing data blocks motion and later enters HOLD. | Position mode only; flight-controller migration; not a mission planner. | Do not switch for #96 |
| **PX4 custom ROS2 mode** | Yes | Potentially | Deep ROS2 customization. | Experimental interface; still needs our planner and full FC requalification. | Future study only |

**Bottom line:** ArduPilot + camera-forward depth gives the most information for the least effort
because the motion constraint fits forward sensing and the FC-side SITL seam already exists.
Build a companion planner only if BendyRuler cannot pass the agreed “detour and resume” cases.

---

## What we proved in SITL

A synthetic 12 m wall is streamed to the FC as `OBSTACLE_DISTANCE` (72×5° sectors, 10 Hz) — the
exact message a real depth-camera or LiDAR bridge would send — then the drone is commanded
*through* it five ways.

![Same wall, different command path — plain GUIDED goto flies through; AUTO mission detours around](avoidance_hero.png)

| Command path | Min clearance | Outcome |
|---|---|---|
| GUIDED goto, no obstacle | — | ✅ baseline, goal reached |
| GUIDED goto through wall (plain) | 0.38 m | ❌ **flew through** — position targets bypass avoidance |
| GUIDED goto + `GUID_OPTIONS=64` | 3.84 m | ✅ detoured around, reached goal |
| GUIDED velocity stream @ 2 m/s | 0.77 m | ✅ stopped before wall (didn't steer around) |
| AUTO waypoint mission | 3.79 m | ✅ detoured around, continued |

![Five SITL runs against the wall](avoidance_summary.png)

**Two Copter 4.5 gotchas to retest on the deployed firmware:**

- Plain **GUIDED position gotos silently bypass path-planning avoidance** unless `GUID_OPTIONS=64`
  is set. Without it, the drone flies straight through obstacles even with the feed live.
- **GUIDED velocity streaming** (the drone-follow pattern) *is* protected — it **stops**, but
  doesn't steer around, and at 2 m/s coasted to within 0.77 m (inside the 2 m margin). Margins
  must be tuned at real flight speeds.

---

## Recommendation

- **Sensor:** use OAK-D as the reference for a generic forward-depth pipeline; select actual
  hardware later. Because the camera always faces motion, add SF45/B only if measured forward-depth
  failures justify wider sensing. Its 320° scan is valuable, but one plane is not all-round 3D.
- **Where the code lives:** the bridge can be a standalone pymavlink daemon (comp-repo stack) or
  a ROS2 node feeding the MAVROS `obstacle_distance` plugin (airside engine) — same MAVLink
  message either way. Start with robust horizontal sectors, retaining timestamp and unknown state.
- **Planner:** compare simple stop and BendyRuler on the exact deployed firmware/command path.
  Add a companion planner only if BendyRuler fails required task-resumption scenarios.
- **Fail closed:** the tested Copter 4.5 stack treats a dead feed as “no obstacles” within seconds.
  A companion watchdog must request BRAKE/LOITER, confirm the command, and alert on stale/missing
  output; retest this failure behavior on the deployed firmware.

---

## Reliability gates (proposed comp-inclusion bar)

1. **Freeze the research contract:** camera faces motion, the sensor interface remains generic,
   and “no collision” is separate from “detour and finish.” Freeze actual hardware, mount,
   firmware/hash, command path, and maximum speed only when qualification starts.
2. **Replay + outdoor bench:** textured/textureless surfaces, foliage, pole/branch/netting,
   reflective/transparent objects if relevant, sun/backlight, roll/pitch, and moving obstacles.
   Measure per-class false-clear/recall, false stops, valid-depth fill, reliable range, and p95/p99
   latency—not only XYZ error.
3. **Fault injection:** frozen frames, USB disconnect, process death/restart, CPU saturation, and
   MAVLink loss. Kill feed → acknowledged BRAKE/LOITER by the agreed deadline; stale is never clear.
4. **Randomized SITL:** sweep speed, margin, latency, and dropout for wall, pop-up/crossing obstacle,
   follow, narrow gap, U-shaped local minimum, and yaw changes. Require zero collisions/breaches.
5. **Staged physical test:** prop-off capture → soft obstacle at low speed → mission speed/lighting
   → combined behavior, with pilot override, geofence, kill path, and rollback.

Cap speed from measured stopping behavior:
`required range = speed × latency + speed²/(2 × braking acceleration) + uncertainty + reserve`.

## Next tickets

Define the generic depth/sector interface → recorded-depth replay + robust sector extractor →
watchdog → SITL qualification → outdoor bench → soft-obstacle flight test → decide whether wider
sensing or a companion planner is actually needed.

---

## Links & references

- **In-repo doc (full detail):** `airside/docs/issue-96-ardupilot-avoidance.md` on branch `jz/96-ardupilot-avoidance`
- **Demo + logs:** `airside/scripts/avoidance/`
- GitHub issue: `UWARG/autonomy-monorepo` #96
- Prior WARG work: `UWARG/obstacle-avoidance` repo + the older *Obstacle Avoidance* page (this page supersedes its direction)
- ArduPilot: [Object Avoidance](https://ardupilot.org/copter/docs/common-object-avoidance-landing-page.html) ·
  [BendyRuler](https://ardupilot.org/copter/docs/common-oa-bendyruler.html) ·
  [Simple avoidance](https://ardupilot.org/copter/docs/common-simple-object-avoidance.html) ·
  [SF45/B](https://ardupilot.org/copter/docs/common-lightware-sf45b.html)
- MAVLink [`OBSTACLE_DISTANCE`](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE)
- OAK-D: [hardware](https://docs.luxonis.com/hardware/products/OAK-D) ·
  [depth limits](https://docs.luxonis.com/hardware/platform/features/depth/) ·
  [ideal-target accuracy](https://docs.luxonis.com/hardware/platform/depth/depth-accuracy/)
- SF45/B: [manufacturer specifications and current price](https://lightwarelidar.com/shop/sf45-b-50-m/)
- PX4: [Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention) ·
  [removed Path Planning Interface](https://docs.px4.io/main/en/computer_vision/path_planning_interface) ·
  [archived PX4-Avoidance](https://github.com/PX4/PX4-Avoidance)

---

*Attachments to upload with this page (drag into the spots above):*
- `avoidance_hero.png` — fly-through vs detour contrast (`airside/scripts/avoidance/figures/`)
- `avoidance_summary.png` — all five runs
- `logs/summaries.txt` — raw run verdicts (optional, as evidence)
