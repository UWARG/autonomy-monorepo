# 3DS - 3D-Scanning a Specified Region via a Coordinated Swarm

`3ds` is a ROS 2 Humble workspace for the 3DS project: mapping a target region/point
of interest using multiple drones in coordination. Currently, the drone takes off,
flies to each setpoint sent from the ground, photographs it, and returns to
where it took off when told to.

## Layout

```text
3ds/
├── compose.yaml
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── src/
│   ├── airside_bringup/      # Launch file that starts the whole system
│   ├── airside_interfaces/   # Custom ROS 2 messages and services
│   ├── camera/               # camera: image stream + photo capture service
│   ├── engine/               # manager (behavior-tree mission), heartbeat
│   └── navigation/           # position_controller
└── warg.toml
```

Packages are grouped by domain; each lists its nodes (executables) above.
Everything is started by `src/airside_bringup/launch/3ds.launch.py`
(`ros2 launch airside_bringup 3ds.launch.py`). The `camera` package's Python
module is `camera_ros`, so it does not shadow the monorepo `camera` library it
wraps.

## Mission

```text
Root [Sequence]
├── ConfigureStreamRates
└── PauseUnlessGuided
    └── Mission [Sequence]
        ├── RecordLaunchPoint             (waits for the pilot to arm)
        ├── Takeoff
        ├── UntilGoHome [Parallel SuccessOnOne]
        │   ├── WaitForGoHome
        │   └── ServeSetpoints [Repeat forever]
        │       └── SkipFailedSetpoint [FailureIsSuccess]
        │           └── ServeSetpoint
        │               ├── WaitForSetpoint
        │               ├── FlyToSetpoint
        │               └── TakePhoto
        ├── LandPhase                     (fly to launch point, land)
        └── MissionComplete [Running]
```

The engine never changes flight mode. `PauseUnlessGuided` waits for the pilot
to select GUIDED, freezes the mission (keeping its progress) whenever the pilot
switches to any other mode, and resumes when GUIDED is selected again. The only
mode change it causes is the final land command, which puts the FCU in LAND.

Once the pilot arms, `RecordLaunchPoint` reads back the FCU home position
(which ArduPilot resets at arming) as the landing spot.

After takeoff the drone hovers and waits for setpoints. Each setpoint is
flown to, photographed, and then the drone waits for the next one. Setpoints
received while busy are queued in order. A setpoint that is not reached
within `WAYPOINT_NAV_TIMEOUT_S`, or whose photo fails, is skipped.

The go-home command preempts everything, including a flight in progress:
queued setpoints are dropped and the drone flies back over the launch point
at `RETURN_ALTITUDE_M` and lands.

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

### Position controller

Behaviors never command MAVROS setpoints directly. They publish where they
want to go to the always-running `position_controller` node (`navigation`
package), which forwards each target to MAVROS. This is the hook for obstacle
avoidance.

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/position_controller/target` | `airside_interfaces/Coordinate` | subscribe | Target `lat`, `lon` and relative `alt` (m) |
| `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | publish | Position-only GUIDED setpoint, relative-altitude frame |

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

### ROS integration inside a behavior

The `BehaviourTree` runner passes `rclpy.Node` as the `node` keyword argument to `setup()`:

```python
def setup(self, **kwargs):
    self._node = kwargs["node"]
    self._sub = self._node.create_subscription(Image, "/camera/image_raw", self._cb, 10)
    self._pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
```

Use `self._node` for all ROS 2 calls (subscriptions, publishers, service clients, action clients, timers).
