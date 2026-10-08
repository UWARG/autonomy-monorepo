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
| `AEAC_CONNECTION_TOKEN` | unset | Secret used by both live AEAC WebSockets; set only in ignored `airside/.env` |
| `AEAC_UAV_ID` | unset | Bidder-defined string in outgoing 1 Hz telemetry |
| `AEAC_OWN_AIRCRAFT_INDEX` | `-1` | Verified server traffic index of this aircraft; live backup fails closed if unset |

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

The default launch starts `triggered_image_publisher` (always running, respawned
on exit), implemented in
`src/camera/camera_ros/triggered_image_publisher_node.py`. It caches `/camera/image_raw`
(`sensor_msgs/Image`), `/mavros/global_position/global` (`sensor_msgs/NavSatFix`),
and `/mavros/imu/data` (`sensor_msgs/Imu`). Sensor topic names can be changed
using ROS remapping.

Send a request after all three sensor feeds are available:

```bash
ros2 topic pub --once /TriggerImageCapture airside_interfaces/msg/TriggerImageCapture "{command: 'capture'}"
```

Groundside must subscribe to `/TriggeredImageCapture` with type
`airside_interfaces/msg/TriggeredImageCapture`. Each accepted request publishes
one message containing the latest image, GPS coordinates and IMU (including
orientation). The outer header records packaging time; the original image and
IMU timestamps are preserved. These are the latest independent readings, not
time-synchronized measurements. Requests with missing inputs are logged and
discarded; send another request once the feeds are ready. Other commands are
ignored.


### ROS integration inside a behavior

The `BehaviourTree` runner passes `rclpy.Node` as the `node` keyword argument to `setup()`:

```python
def setup(self, **kwargs):
    self._node = kwargs["node"]
    self._sub = self._node.create_subscription(Image, "/camera/image_raw", self._cb, 10)
    self._pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
```

Use `self._node` for all ROS 2 calls (subscriptions, publishers, service clients, action clients, timers).

### Obstacle-aware lapping

The lapping subtree uses `ObstacleAwareFlyToWaypoint`. It keeps ArduPilot in
`GUIDED`, consumes a 360-degree `sensor_msgs/LaserScan` on
`/obstacle_avoidance/scan`, and publishes ENU `geometry_msgs/TwistStamped`
commands on `/mavros/setpoint_velocity/cmd_vel` at 10 Hz. No global-position
setpoint is published while this behavior is active.

The scan frame must be `base_link`. Finite ranges are obstacles and positive
infinity is observed clear space. Empty, partial, stale, future-dated, frozen,
wrong-frame, NaN, negative-infinity, or otherwise invalid scans are fail-closed:
the behavior publishes zero velocity and remains `RUNNING`. Stale pose, GPS,
altitude, or FC state has the same result. If the pilot leaves `GUIDED` or the
vehicle is disarmed, the behavior releases setpoint ownership and publishes
nothing until fresh scan and telemetry have arrived after returning to
`GUIDED`.

Planner state is published as `diagnostic_msgs/DiagnosticArray` on
`/obstacle_avoidance/diagnostics`. The production scan adapter must honor this
contract; the synthetic SITL publisher under
`experiments/obstacle_avoidance/` is for
qualification only and is not an OAK-D integration.

The #144/#181 research material, SITL runners, qualification harness, plots,
replay page, and flight-test Compose override are collected in
[`experiments/obstacle_avoidance/`](experiments/obstacle_avoidance/README.md).

The primary [live AEAC → BendyRuler2D path](experiments/obstacle_avoidance/aeac-bendy-ruler-flight-test.md)
uses `compose.aeac-bendy.yaml`. It feeds complete server snapshots to
`/aeac/live_traffic` and sends this UAV's telemetry at 1 Hz on a separate
WebSocket. The synthetic static-obstacle launch still uses `/aeac/traffic`.
The isolated A* launch on this branch is experimental; Honzik's #203 remains
the separate fallback branch. Neither live path is approved for a physical
flight until the real-server props-off checks pass.
