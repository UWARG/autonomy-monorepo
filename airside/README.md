# Airside

`airside` is a ROS 2 Humble workspace for running the overall auto airside architecture.

## Layout

```text
airside/
├── compose.yaml
├── compose.jetson.yaml       # Jetson Orin override (GPU runtime + VSLAM service)
├── docker/
│   ├── Dockerfile
│   ├── Dockerfile.vslam      # Isaac ROS Visual SLAM (Jetson only)
│   └── airside_entrypoint.sh
├── src/
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

## Jetson Orin Nano

`compose.jetson.yaml` is a Docker Compose override that enables the NVIDIA container runtime and adds the Isaac ROS Visual SLAM service. Use it on the Jetson Orin Nano in addition to the base compose file.

### Jetson prerequisites

- JetPack 6 installed on the Jetson
- `nvidia-container-toolkit` configured as the default Docker runtime (included with JetPack)

### Running on Jetson

```bash
docker compose -f compose.yaml -f compose.jetson.yaml up -d
```

This starts two services:

- **airside** — the behavior tree engine, with GPU access via the NVIDIA runtime
- **vslam** — Isaac ROS Visual SLAM node, publishing odometry on `/visual_slam/tracking/odometry`

Both services use `network_mode: host` so ROS 2 DDS topic discovery works across containers without extra configuration.

### Isaac ROS Visual SLAM

The VSLAM service (`Dockerfile.vslam`) installs `ros-humble-isaac-ros-visual-slam` from NVIDIA's Isaac ROS apt repository. It uses the GPU (cuVSLAM) and subscribes to rectified stereo image topics:

| Topic                        | Description                  |
| ---------------------------- | ---------------------------- |
| `visual_slam/image_0`        | Left rectified image         |
| `visual_slam/image_1`        | Right rectified image        |
| `visual_slam/camera_info_0`  | Left camera calibration      |
| `visual_slam/camera_info_1`  | Right camera calibration     |
| `visual_slam/imu`            | IMU data (optional)          |

These topics must be provided by a camera driver running on the same ROS domain (e.g. the `depthai_ros_driver` for OAK-D). Remap the driver's output topics onto the names above, or pass `image_topic_name_` launch arguments to the VSLAM node.

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
