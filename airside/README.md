# Airside

`airside` is a ROS 2 Humble workspace for running the overall auto airside architecture.

## Layout

```text
airside/
├── compose.yaml
├── docker/
│   ├── Dockerfile
│   └── airside_entrypoint.sh
├── src/
│   ├── airside_bringup/      # Launch file that starts the whole system
│   ├── airside_interfaces/   # Custom ROS 2 messages
│   ├── camera/               # camera, triggered_image_publisher
│   ├── engine/               # manager (behavior-tree mission), rc_bridge, heartbeat + mission config
│   └── navigation/           # position_controller
└── warg.toml
```

Packages are grouped by domain; each lists its nodes (executables) above.
Everything is started by `src/airside_bringup/launch/airside.launch.py`
(`ros2 launch airside_bringup airside.launch.py`).

The `camera` package's Python module is `camera_ros`, so it does not shadow the
monorepo `camera` library it wraps.

## Prerequisites

- Docker

## Usage

All commands are available through the warg CLI from the repo root, or by
running `docker compose` directly inside `airside/`.

### Via warg CLI

```bash
warg run airside build          # Build the Docker image
warg run airside compose-up     # Start the engine service (detached)
warg run airside compose-down   # Stop the engine service
warg run airside logs           # Follow service logs
warg run airside test           # Run the test suite
warg run airside shell          # Open an interactive shell in the container
```

### Via docker compose

```bash
docker compose build
docker compose up -d
docker compose down
docker compose logs -f
docker compose run --rm airside bash
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

#### Mission

1. **Record launch point**: once the pilot arms, the FCU home position (which
   ArduPilot resets at arming) is read back and stored as the landing spot.
2. **Lapping**: fly the waypoints in `config/waypoints.yaml` as a clockwise
   sweep, starting from the side facing the launch point, until the lapping
   deadline.
3. **Reconnaissance**: the engine commands nothing and waits for the pilot to
   flip the recon-complete RC switch (`RECON_COMPLETE_RC_CHANNEL`). The pilot
   flies (switching out of GUIDED pauses the mission) and captures images on
   manual triggers through `triggered_image_publisher`. Switching back to
   GUIDED with the switch flipped continues to the land phase.
4. **Land**: fly back over the launch point at `RETURN_ALTITUDE_M` and land.

The engine never changes flight mode. The whole mission is wrapped in
`PauseUnlessGuided`: it waits for the pilot to select GUIDED, freezes (keeping
its progress) whenever the pilot switches to any other mode, and resumes when
GUIDED is selected again. The only mode change it causes is the final land
command, which puts the FCU in LAND.

#### Position controller

Behaviors never command MAVROS setpoints directly. They publish where they
want to go to the always-running `position_controller` node, which forwards
each target to MAVROS.

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/position_controller/target` | `airside_interfaces/Coordinate` | subscribe | Target `lat`, `lon` and relative `alt` (m) |
| `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | publish | Position-only GUIDED setpoint, relative-altitude frame |

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

### Triggered image capture

The `camera` node (in the `camera` package) publishes on `camera/image_raw`,
selecting a real or simulated driver via the `camera_type` ROS parameter
(`sim` | `oakd` | `arducam`, default `sim`). The launch file sets it to `oakd`;
for bench testing without hardware, run the node standalone with
`ros2 run camera camera --ros-args -p camera_type:=sim`.

The `triggered_image_publisher` node (in the `camera` package, always running,
respawned on exit) caches the latest message from each input (via
`message_filters.Cache`) and returns them as one bundle from the
`/capture_image` service (`airside_interfaces/srv/CaptureImage`):

| Name | Type | Kind | Purpose |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | subscribe | Camera feed |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | subscribe | GPS fix |
| `/mavros/imu/data` | `sensor_msgs/Imu` | subscribe | Orientation |
| `/capture_image` | `airside_interfaces/srv/CaptureImage` | service | Returns image, GPS location and IMU |

```bash
ros2 service call /capture_image airside_interfaces/srv/CaptureImage
```

It's a service rather than a trigger/response topic pair because groundside
reaches it through rosbridge, which subscribes to topics best-effort and drops
large one-off messages; service replies are reliable. The response header
records packaging time; the original image, GPS and IMU timestamps are
preserved. These are the latest independent readings, not time-synchronized
measurements. If an input hasn't published yet, the response has
`success: false` and `message` names what's missing.

### ROS integration inside a behavior

The `BehaviourTree` runner passes `rclpy.Node` as the `node` keyword argument to `setup()`:

```python
def setup(self, **kwargs):
    self._node = kwargs["node"]
    self._sub = self._node.create_subscription(Image, "/camera/image_raw", self._cb, 10)
    self._pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
```

Use `self._node` for all ROS 2 calls (subscriptions, publishers, service clients, action clients, timers).
