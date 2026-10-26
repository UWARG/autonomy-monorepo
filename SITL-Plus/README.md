# SITL-Plus

A software-in-the-loop simulation environment using PyBullet for physics, ArduPilot SITL for flight control, and Rerun visualization.

![SITL-Plus Rerun demo](gifs/Adobe%20Express%20-%202026-07-07%2000-23-54.gif)

## Architecture

The simulation is split across processes:

| Process | Where it runs | What it does |
|---|---|---|
| PyBullet sim (`main.py`) | Docker container | Physics engine, camera & range-finder simulation, Rerun logging |
| ArduPilot SITL (`sim_vehicle.py`) | Docker container | Flight controller, MAVLink on TCP port 5761 |
| Rerun Viewer (`uv run rerun`) | Host | Displays logged sensor data and drone pose |
| `rerun_airside.py` | Host | Drives the mission via MAVLink |

Sensor data is simulated in the container and logged to Rerun over gRPC via `host.docker.internal`. UDP ports are still used internally:

- Camera frames: ports **6000** (downward) and **6002** (forward)
- Range finder: port **6004**
- Telemetry (position/attitude): port **4000**

## How it works

SITL-Plus replaces ArduPilot’s built-in physics with a PyBullet world, then feeds ArduPilot IMU/pose over the [JSON SITL model](https://ardupilot.org/dev/docs/sitl-with-JSON.html) interface while streaming sensors to Rerun (and optionally to host-side airside consumers).

### Control loop (PyBullet ↔ ArduPilot)

1. ArduPilot SITL runs with `--model JSON:127.0.0.1` and talks to `main.py` on **UDP 9002**.
2. Each tick, SITL sends motor **PWM**. `main.py` applies thrust/torque to the Iris URDF in PyBullet, steps the physics (`SIM_RATE_HZ`, default **800**), and replies with gyro, accel, position, Euler angles, and velocity.
3. Vectors/quaternions are converted from PyBullet’s frame into ArduPilot’s NED-style frame before being packed into the JSON reply (`vector_to_AP` / `quaternion_to_AP` in `src/main.py`).
4. The FC runs as normal ArduCopter; `rerun_airside.py` on the host connects over **TCP 5761** (MAVLink) to upload missions / command the vehicle. Compose also publishes **14550** for GCS tools (e.g. Mission Planner).

The container entrypoint starts PyBullet first, waits briefly, then launches `sim_vehicle.py` so the JSON socket is ready.

### Sensors

| Sensor | Implementation | Output |
|---|---|---|
| Cameras (`6000` down, `6002` forward) | `p.getCameraImage` on the Iris body (224×224, FOV 60°) | JPEG RGB + PNG depth over UDP; Rerun `EncodedImage` / `DepthImage` |
| Range finder (`6004`) | PyBullet ray cast along body axis | Distance (`float`) over UDP; Rerun time series |
| Drone pose | Base link pose each physics step | Rerun `Transform3D` on entity `drone` |

Depth is stored in centimetres (`uint16`) and logged with `meter=100` so Rerun displays metres. UDP packets to `SENSOR_HOST` (compose default: `host.docker.internal`) carry a header of RGB length, depth length, far, and near, then the encoded payloads—so airside / `rerun_airside.py` can decode the same frames the viewer shows.

Camera threads and the physics loop run concurrently; scene props (plane, barrels, hoop, etc.) are spawned in `main.py` for visual/ranging targets.

### Visualization (Rerun)

- App id: `SITL-Plus` (`rr.init` in `main.py`)
- Transport: gRPC to `rerun+http://host.docker.internal:9876/proxy` (viewer on the **host**, `uv run rerun`)
- Entities: `{port}_rgb_image`, `{port}_depth_map`, range streams, and the `drone` transform

If a saved viewport layout gets stuck, reset the blueprint in the viewer or run `uv run rerun reset` on the host.

### Why Docker + host split

ArduPilot and the PyBullet model stay in Linux (image build clones ArduPilot and runs `install-prereqs-ubuntu.sh` + `waf` for the SITL board). The Rerun viewer and mission script stay on the host so you get a native GUI and can point UDP/MAVLink at Windows or WSL without nesting displays in the container.

## Build

Build the image once:

```bash
docker compose build
```

## Run

terminal 1 — start the Rerun viewer:
```bash
uv run rerun
```

terminal 2 — start the container (headless PyBullet + ArduPilot SITL):
```bash
docker compose up
```

terminal 3 — mission controller:
```bash
warg run SITL-Plus rerun_airside
```

## Local SITL (no Docker)

Follow the ArduPilot [Linux build guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux). Once set up, run this from the `ardupilot` directory in WSL:

```bash
python3 ./Tools/autotest/sim_vehicle.py -N -v ArduCopter -f quad \
  --model JSON:<YOUR_IPV4_ADDR> --console --map \
  --out tcpin:0.0.0.0:5761
```

Replace `<YOUR_IPV4_ADDR>` with your WSL host's IPv4 address. Also set `SENSOR_HOST` to `127.0.0.1` in your environment when running `rerun_airside` locally.

## Logs

Container logs are written to `./logs/` (mounted into the container):
- `logs/pybullet.log` — PyBullet / `main.py` output
- `logs/sim_vehicle.log` — ArduPilot SITL / MAVProxy output
