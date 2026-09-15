# Non-Optimal Precision Landing

Visual teach-and-repeat precision landing for the **non-optimal / Jetson** profile.
The idea is simple: while climbing away from the pad, remember what the ground
looked like at each altitude; when coming back to land, match the live downward
camera against that memory and steer until the vehicle is back over the launch
point.

The pipeline runs on ROS 2 (Humble), OpenCV, and MAVROS against ArduPilot. It has
been flight-tested on an NVIDIA Jetson and validated in **SITL-Plus**, a custom
software-in-the-loop environment that bridges PyBullet physics with ArduPilot
SITL.

## Mission sequence

The Jetson behavior tree (`manager_jetson`) is a one-shot sequence:

1. **Takeoff** (`precision_takeoff`) — sends the `/takeoff` action. While the
   vehicle climbs, the processor builds the teach map and returns the launch
   GPS fix on the blackboard for later RTL.
2. **Fly around** — waits on RC channel 6 so a pilot (or script) can displace
   the aircraft away from the pad before autonomous return begins.
3. **Return to launch** — flies a global setpoint back to the stored launch
   latitude / longitude / altitude so visual landing starts near the teach site
   rather than from an arbitrary offset.
4. **Landing** — sends the `/landing` action. The processor drives vision-based
   setpoints until it hands the aircraft to ArduPilot `LAND` for touchdown.

Launch entry points: `warg run airside run-jetson`,
`docker compose --profile jetson up`, or
`docker compose -f compose.jetson-sitl.yaml up` (shared network with SITL-Plus).

`engine_jetson.launch.py` brings up MAVROS, the camera node, `manager_jetson`,
`processor`, `controller`, and `rc_node`.

## Teach phase (ascent)

While a takeoff goal is active, `processor` runs a 10 Hz timer that:

1. Reads the downward camera, IMU attitude, and rangefinder AGL (minus a small
   camera-offset constant).
2. Undistorts the frame with calibrated intrinsics, converts to grayscale, and
   extracts up to 1000 **ORB** keypoints / descriptors (Oriented FAST and
   Rotated BRIEF).
3. Rejects frames with fewer than **150** keypoints. Sparse teach frames are
   worse than missing altitudes: the repeat pass would select them by height,
   fail to match, and stall. Rejected altitudes are retried on the next tick
   instead of being stored.
4. On acceptance, stores `(keypoints, descriptors, roll, pitch, yaw)` in an
   altitude-keyed `SortedDict`, advances the last-captured altitude, and writes
   `takeoff_<agl>.png` under `/images` (mounted to `airside/src/images/` on the
   host).

Capture spacing is altitude-adaptive from the rangefinder: denser near the
ground (`~0.1 m` steps below 1 m AGL) and coarser higher up (`~0.25 m`). Takeoff
completes once the map has been filled up to the configured top altitude
(`last_image_altitude`, default 7.5 m).

## Repeat phase (descent)

When a landing goal is active and the teach map is non-empty:

1. Look up the teach entry whose altitude is nearest-at-or-below current AGL
   (`bisect_right` on the sorted map).
2. Extract live ORB features and match them to the teach descriptors with a
   Hamming **BFMatcher** (CUDA on Jetson when available, CPU otherwise) using
   kNN (`k=2`).
3. Apply **Lowe’s ratio test** (default ratio `0.55`). Ground texture often
   produces near-identical descriptors, so a small Hamming distance alone is not
   enough; a match is kept only when the best candidate is clearly better than
   its runner-up. Fewer than 10 survivors → publish an invalid error and wait.
4. Back-project matched pixels into a metric ground plane with
   `pixel_to_3d(...)`, compensating for roll/pitch and using teach altitude for
   teach points / live AGL for live points. The processor’s ground frame is
   `+x = left`, `+y = forward`.
5. Estimate a 2D **similarity transform** with
   `cv2.estimateAffinePartial2D(..., method=RANSAC)` (rotation + uniform scale +
   translation). Reject if RANSAC fails, the inlier ratio is below `0.4`, or the
   recovered scale wanders outside `[0, 2]` (degenerate altitude mismatch).
6. Publish a `custom_interfaces/Error` with lateral translation (`x`, `y`),
   measured rotation, IMU yaw error vs the teach yaw, and a tapered descent
   rate `vz`.

Debug artifacts: `landing_<alt>.png` and a side-by-side
`landing_overlay_<alt>.png` (teach | live with a correction arrow).

## Alignment cone and descent authority

Descent is not hard-gated by a fixed XY tolerance. Instead an
**altitude-proportional alignment cone** sets

```text
align_tolerance = max(0.05 m, 0.15 * AGL)
taper           = clamp(2 - xy_error / align_tolerance, 0, 1)
vz              = 0.1 m/s * taper
```

Near the pad the vehicle must be tightly centered before much downward speed is
allowed; higher up, more lateral error is tolerated and descent still progresses.
When `xy_error` exceeds roughly `2 × align_tolerance`, `vz` reaches zero and the
controller holds altitude while correcting laterally / in yaw.

Below ~1 m AGL (or below the bottom of the teach map), vision hands off to
ArduPilot **`LAND`** via MAVROS `set_mode`. The processor then watches
`/mavros/extended_state` and declares success after several consecutive
on-ground samples so a single spurious reading cannot end the action early.

## Stale-vision recovery

If matching has been unavailable for more than **2 s**, holding position cannot
recover a fix (the view never changes). The processor then:

- **Commits** a blind descent if the last valid fix was inside the full-descent
  core of the cone and AGL ≤ 1 m, or
- **Climbs** slowly (`vz = -0.1` in the processor’s sign convention) to widen the
  camera footprint and re-acquire, unless already above the top of the teach map
  (in which case climbing cannot help).

## Controller

`controller` subscribes to `/error` and publishes body-frame velocity setpoints
on `/mavros/setpoint_raw/local` (`PositionTarget`, FRAME_BODY_NED) at the vision
rate:

- Independent PI loops for lateral X/Y and yaw rate (anti-windup, output clamps).
- Lateral axes are remapped from the processor ground frame into the drone body
  frame expected by MAVROS.
- `vz` comes straight from the processor’s tapered descent command.
- Invalid errors zero lateral demand but still apply `vz` / yaw (used by
  stale-vision climb / commit).
- `landing_complete` zeros all setpoints, resets integrators, and stops
  commanding so GUIDED setpoints do not fight ArduPilot `LAND`.

## Validation

- **SITL-Plus** (`../SITL-Plus`): PyBullet physics + ArduPilot SITL, with camera /
  rangefinder streaming into the airside stack over the shared Docker network
  (`compose.jetson-sitl.yaml`, `FCU_URL=tcp://sitl-plus:5761`).
- **Hardware**: NVIDIA Jetson deployment with CUDA matcher when available; camera
  and rangefinder via the Jetson launch graph.

| Piece | Where |
|---|---|
| Jetson / precision-landing tree | `src/engine/engine/manager_jetson.py` |
| Jetson launch graph | `src/engine/launch/engine_jetson.launch.py` |
| Vision teach/repeat processor | `src/nodes/nodes/processor.py` |
| Descent PI controller | `src/nodes/nodes/controller.py` |
| Precision takeoff behavior | `src/engine/engine/behaviors/navigation/precision_takeoff.py` |
| Landing behavior | `src/engine/engine/behaviors/navigation/landing.py` |
| Teach / overlay images | `src/images/` (container `/images`) |
| Jetson compose profile | `docker compose --profile jetson up` / `warg run airside run-jetson` |
| Shared-network SITL | `compose.jetson-sitl.yaml` |

# Airside

`airside` is a ROS 2 Humble workspace for running the overall auto airside architecture.

## Layout

```text
airside/
├── compose.yaml
├── docker/
│   ├── Dockerfile
│   ├── Jetson.Dockerfile
│   └── airside_entrypoint.sh
├── src/
│   ├── airside_interfaces/
│   ├── engine/
│   └── wrapper/
└── warg.toml
```

## Prerequisites

- Docker

## Usage

All commands are available through the warg CLI from the repo root, or by
running `docker compose` directly inside `airside/`.

### Via warg CLI

```bash
warg run airside build                 # Build the Docker image
warg run airside compose-up            # Default profile (detached)
warg run airside run-jetson            # Jetson / precision landing profile
warg run airside compose-down          # Stop services
warg run airside logs                  # Follow service logs
warg run airside test                  # Run the test suite
warg run airside shell                 # Open an interactive shell in the container
```

### Via docker compose

```bash
docker compose build
docker compose --profile default up          # competition BT
docker compose --profile jetson up           # teach / fly-around / RTL / visual land
docker compose --profile default --profile jetson down
docker compose logs -f
docker compose --profile default run --rm airside bash
```

## Adding a monorepo library

To expose a new monorepo library (e.g. `camera/`) inside the container, add the following lines to the dockerfile:

**`airside/docker/Dockerfile`**
```dockerfile
COPY camera/ /monorepo/camera/
RUN pip install /monorepo/camera
```

## Configuration

| Environment variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain ID for DDS discovery isolation |
| `MAP_MANAGER_DATA_DIR` | `/ros_ws/data` | Directory where the map manager stores target logs (mounted to `airside/data/` on the host) |
| `FCU_URL` | `serial:///dev/serial0:115200` | MAVROS connection to the ArduPilot FCU. SITL: see `compose.sitl.yaml` |

### Networking

The container runs with `network_mode: host` to ensure direct access
to the host's network interfaces. On Docker Desktop (macOS/Windows), enable
host networking under Settings > Resources > Network, or switch to the 1:1
`ports:` fallback commented in `compose.yaml` if needed.

### Logs

ROS logs (rclpy logger output and captured node stdout) are written to
`airside/log/ros/` on the host via the `ROS_LOG_DIR` mount in `compose.yaml`.

## Developer Guide

### Behavior tree

The engine is built with [py_trees_ros](https://py-trees-ros.readthedocs.io/en/latest/).  The tree is composed in `src/engine/engine/manager.py` and ticked every `TICK_PERIOD_MS` milliseconds.

#### Adding a behavior

1. Copy `src/engine/engine/behaviors/template.py`, rename it, and implement the lifecycle methods.
2. Add the new behavior as a child of the root (or a composite) in `create_root()` inside `manager.py`.

#### Behavior lifecycle

| Method | When called | Purpose |
|---|---|---|
| `setup(**kwargs)` | Once, during `tree.setup()` | Create ROS resources (subscriptions, publishers, action clients) |
| `initialise()` | Each time the behavior transitions from IDLE to RUNNING | Reset internal state |
| `update()` | Every tick while RUNNING | Evaluate conditions; return `RUNNING`, `SUCCESS`, or `FAILURE` |
| `terminate(new_status)` | Whenever the behavior exits | Cancel in-behavior actions |

#### Composite types

| Type | Behaviour |
|---|---|
| `Sequence` | Ticks children left-to-right; returns FAILURE on the first FAILURE child, SUCCESS only when all children succeed |
| `Selector` | Ticks children left-to-right; returns SUCCESS on the first SUCCESS child, FAILURE only when all children fail |
| `Parallel` | Ticks all children every tick; uses a `SuccessOnAll` or `SuccessOnOne` policy |

### Blackboard

py_trees provides a global key-value store (the *Blackboard*) shared by all behaviors in the tree.

#### Declaring and using keys

```python
# In __init__
self.blackboard = self.attach_blackboard_client(name=self.name)
self.blackboard.register_key(key="altitude", access=py_trees.common.Access.READ)
self.blackboard.register_key(key="waypoint",  access=py_trees.common.Access.WRITE)

# In initialise / update / terminate
alt = self.blackboard.altitude
self.blackboard.waypoint = (lat, lon)
```

#### Access levels

| Level | Description |
|---|---|
| `READ` | This client may only read the key |
| `WRITE` | This client may read and write |
| `EXCLUSIVE_WRITE` | This client may read and write; all other clients are blocked from writing |

#### Namespacing

Prefix keys with `/`:

```python
self.blackboard.register_key(key="/perception/target", access=py_trees.common.Access.WRITE)
```

#### Setting initial values

To set initial values, create a client in `manager.py` before tree setup:

```python
blackboard = py_trees.blackboard.Client(name="init")
blackboard.register_key(key="altitude", access=py_trees.common.Access.WRITE)
blackboard.altitude = 0.0
```

### Map manager

The `map_manager` node (in the `wrapper` package) is launched alongside the engine and records detected targets for post processing.

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/capture/target_location` | `airside_interfaces/Target` | subscribe | A detected target: `colour` (a `utils.src.enums.Colours` member name, e.g. `"RED"`) and `location` (`airside_interfaces/Coordinate`: `lat`, `lon`, `alt`) |
| `/trigger_post_processing` | `std_msgs/Empty` | subscribe | Snapshots the current target log to a timestamped file for post processing |

Received targets are appended to `$MAP_MANAGER_DATA_DIR/targets.jsonl`, which is wiped at every startup (one file per run). Each trigger copies it to `targets_<YYYY-MM-DDTHH-MM-SS>.jsonl` in the same directory.

### ROS integration inside a behavior

The `BehaviourTree` runner passes `rclpy.Node` as the `node` keyword argument to `setup()`:

```python
def setup(self, **kwargs):
    self._node = kwargs["node"]
    self._sub = self._node.create_subscription(Image, "/camera/image_raw", self._cb, 10)
    self._pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
```

Use `self._node` for all ROS 2 calls (subscriptions, publishers, service clients, action clients, timers).
