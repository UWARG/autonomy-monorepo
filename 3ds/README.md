# 3DS

`3ds` is a ROS 2 Humble workspace for the 3DS mission: the drone takes off,
flies to each setpoint sent from the ground, photographs it, and returns home
when told to. It is derived from [`airside`](../airside/README.md).

## Layout

```text
3ds/
├── compose.yaml
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── src/
│   ├── airside_interfaces/   # Messages and services
│   ├── camera_driver/        # Camera stream + photo capture service
│   ├── engine/               # Behavior tree, RC bridge, heartbeat, launch file
│   ├── ground_comms/         # Sensor fusion for the ground station
│   └── target_mapping/       # Target log + building-relative target localizer
└── warg.toml
```

## Mission

```text
KillSwitch
└── Mission [Sequence]
    ├── ConfigureStreamRates
    ├── LoadHome                      (src/engine/config/home.yaml)
    ├── Takeoff
    ├── UntilGoHome [Parallel SuccessOnOne]
    │   ├── WaitForGoHome
    │   └── ServeSetpoints [Repeat forever]
    │       └── SkipFailedSetpoint [FailureIsSuccess]
    │           └── ServeSetpoint
    │               ├── WaitForSetpoint
    │               ├── FlyToSetpoint
    │               └── TakePhoto
    ├── LandPhase                     (fly home, land)
    └── MissionComplete [Running]
```

After takeoff the drone hovers and waits for setpoints. Each setpoint is
flown to, photographed, and then the drone waits for the next one. Setpoints
received while busy are queued in order. A setpoint that is not reached
within `WAYPOINT_NAV_TIMEOUT_S`, or whose photo fails, is skipped.

The go-home command preempts everything, including a flight in progress:
queued setpoints are dropped and the drone flies to the home coordinate and
lands.

| Topic / service | Type | Direction | Purpose |
|---|---|---|---|
| `/mission/setpoint` | `airside_interfaces/Coordinate` | subscribe | Next location to photograph (`lat`, `lon`, relative `alt` in meters, at least `SETPOINT_MIN_ALTITUDE_M`) |
| `/mission/go_home` | `std_msgs/Empty` | subscribe | Return home and land |
| `/camera/capture` | `airside_interfaces/CapturePhoto` | service (camera node) | Saves the latest camera frame, tagged with the setpoint and measured position |

From the ground (or inside the container):

```bash
ros2 topic pub --once /mission/setpoint airside_interfaces/msg/Coordinate "{lat: 43.4339, lon: -80.5776, alt: 10.0}"
ros2 topic pub --once /mission/go_home std_msgs/msg/Empty
```

Both topics are also reachable over rosbridge on port 9090.

### Photos

Each camera node start creates `$PHOTO_DATA_DIR/run_<YYYY-MM-DDTHH-MM-SS>/`
(mounted to `3ds/data/photos/` on the host) containing `photo_NNNN.png` files
and a `photos.jsonl` index with one record per photo: `stamp`, `file`,
`setpoint` and `position` (each `lat`, `lon`, `alt`; `position` is NaN if no
GPS fix was available).

## Prerequisites

- Docker

## Usage

All commands are available through the warg CLI from the repo root, or by
running `docker compose` directly inside `3ds/`.

### Via warg CLI

```bash
warg run 3ds build          # Build the Docker image
warg run 3ds compose-up     # Start the engine service (detached)
warg run 3ds compose-down   # Stop the engine service
warg run 3ds logs           # Follow service logs
warg run 3ds test           # Run the test suite
warg run 3ds shell          # Open an interactive shell in the container
```

### Via docker compose

```bash
docker compose build
docker compose up -d
docker compose down
docker compose logs -f
docker compose run --rm 3ds bash
```

## Adding a monorepo library

To expose a new monorepo library (e.g. `camera/`) inside the container, add the following lines to the dockerfile:

**`3ds/docker/Dockerfile`**
```dockerfile
COPY camera/ /monorepo/camera/
RUN pip install /monorepo/camera
```

## Configuration

| Environment variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain ID for DDS discovery isolation |
| `MAP_MANAGER_DATA_DIR` | `/ros_ws/data` | Directory where the map manager stores target logs (mounted to `3ds/data/` on the host) |
| `PHOTO_DATA_DIR` | `/ros_ws/data/photos` | Directory where the camera node saves captured photos |
| `FCU_URL` | `serial:///dev/serial0:115200` | MAVROS connection to the ArduPilot FCU. SITL: see `compose.sitl.yaml` |

### Networking

The container runs with `network_mode: host` to ensure direct access
to the host's network interfaces. On Docker Desktop (macOS/Windows), enable
host networking under Settings > Resources > Network, or switch to the 1:1
`ports:` fallback commented in `compose.yaml` if needed.

### Logs

ROS logs (rclpy logger output and captured node stdout) are written to
`3ds/log/ros/` on the host via the `ROS_LOG_DIR` mount in `compose.yaml`.

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

The `map_manager` node (in the `target_mapping` package) is launched alongside the engine and records detected targets for post processing.

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/capture/target_location` | `airside_interfaces/Target` | subscribe | A detected target: `colour` (a `utils.src.enums.Colours` member name, e.g. `"RED"`) and `location` (`airside_interfaces/Coordinate`: `lat`, `lon`, `alt`) |
| `/trigger_post_processing` | `std_msgs/Empty` | subscribe | Snapshots the current target log to a timestamped file for post processing |

Received targets are appended to `$MAP_MANAGER_DATA_DIR/targets.jsonl`, which is wiped at every startup (one file per run). Each trigger copies it to `targets_<YYYY-MM-DDTHH-MM-SS>.jsonl` in the same directory.

### Building-space target localizer

The `building_target_localizer` node (in the `target_mapping` package) converts processed building planes and
target points into firefighter-readable location descriptions. The non-ROS
geometry and localization code lives in the top-level `perception` project.

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/processed_map` | `airside_interfaces/ProcessedMap` | subscribe | Complete building geometry and target snapshot |
| `/targets_located` | `std_msgs/String` | publish | Newline-separated final target descriptions |

Both topics use reliable, transient-local, depth-one QoS. Input geometry must
use `header.frame_id = "mission_frd"`, where +x is north/forward, +y is
east/right, and +z is down. Plane normals point out of the building and use
`normal · point + offset = 0`.

Each rectangular wing references two adjacent observed wall planes and gives
the distance to each opposing wall. The node completes those walls, merges
connected wings, removes internal faces, and derives outer and inside corners.
All wings share the supplied ground plane and building height. Disconnected
wings and footprints containing holes are rejected at snapshot level.

Targets that cannot be snapped safely or have ambiguous geometry are omitted
from the description string and logged by the node. Other targets in the
same snapshot continue to be reported. Plane covariance is row-major for
`[normal.x, normal.y, normal.z, offset]`; target covariance is row-major for
`[x, y, z]`. Each description includes the propagated 95th-percentile error.

The default node parameters are:

| Parameter | Default |
|---|---:|
| `expected_frame_id` | `mission_frd` |
| `max_snap_distance_m` | `0.5` |
| `surface_tie_tolerance_m` | `0.1` |
| `near_wall_distance_m` | `5.0` |
| `anchor_tie_tolerance_m` | `0.25` |
| `wall_vertical_tolerance_deg` | `5.0` |
| `wing_orthogonality_tolerance_deg` | `5.0` |
| `wing_join_tolerance_m` | `0.05` |
| `condition_epsilon` | `1e-6` |
| `uncertainty_samples` | `1000` |
| `uncertainty_seed` | `97` |
| `max_unstable_sample_fraction` | `0.05` |

### ROS integration inside a behavior

The `BehaviourTree` runner passes `rclpy.Node` as the `node` keyword argument to `setup()`:

```python
def setup(self, **kwargs):
    self._node = kwargs["node"]
    self._sub = self._node.create_subscription(Image, "/camera/image_raw", self._cb, 10)
    self._pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
```

Use `self._node` for all ROS 2 calls (subscriptions, publishers, service clients, action clients, timers).
