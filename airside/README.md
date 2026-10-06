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
each target to MAVROS, steering around other aircraft on the way.

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/position_controller/target` | `airside_interfaces/Coordinate` | subscribe | Target `lat`, `lon` and relative `alt` (m) |
| `/position_controller/obstacle` | `airside_interfaces/Obstacle` | subscribe | One aircraft to keep away from, keyed by `aircraft_index` |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | subscribe | The drone's own position, for obstacle avoidance |
| `/mavros/setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | publish | Position-only GUIDED setpoint, relative-altitude frame |

##### Obstacle avoidance

Each `Obstacle` message replaces the previous one with the same
`aircraft_index`; until the next one arrives the obstacle stays frozen where it
was last reported. Nothing in this workspace publishes the topic yet.

Every obstacle gets a keep-away zone: a circle of `horizontal_keep_away` plus
`KEEP_AWAY_MARGIN_M`, stretched forwards along its `direction` by `speed` x
`FORWARD_ZONE_LOOKAHEAD_S` (at most `FORWARD_ZONE_MAX_EXTENSION_M`), so the side
it is flying towards is kept clearer than the side it is leaving. Zones are
horizontal only and apply at every altitude.

For each target, the planner (`src/navigation/navigation/visibility_graph.py`)
checks the whole straight line to it. If that is clear the target is forwarded
untouched. Otherwise it places corner points around every zone, joins up the
drone, the target and the corners that can see each other without crossing a
zone, and finds the shortest way through with A*. The setpoint sent is the first
corner on that path, so the drone turns as soon as a zone is in its way, however
far off. Special cases:

- Drone already inside a zone: it flies straight out of it.
- Target inside a zone: it waits at the edge of the zone, as close to the target
  as allowed.
- No way through: it holds position.

A path is only re-planned when a target arrives, so behaviors must keep
publishing their target while flying.

All tuning constants are in `src/navigation/navigation/constants.py`.

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
