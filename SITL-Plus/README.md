# SITL-Plus

A software-in-the-loop simulation environment using PyBullet for physics, ArduPilot SITL for flight control, and Rerun visualization.

![SITL-Plus Rerun demo](gifs/Adobe%20Express%20-%202026-07-07%2000-23-54.gif)

## Architecture

The simulation is split across processes:

| Process | Where it runs | What it does |
|---|---|---|
| PyBullet sim (`main.py`) | Docker container | Physics engine, camera & range-finder simulation, Rerun gRPC server |
| ArduPilot SITL (`sim_vehicle.py`) | Docker container | Flight controller, MAVLink on TCP port 5761 |
| Rerun Viewer (host) | Host | Connects to the container's Rerun server and displays sensor data / drone pose |
| `rerun_airside.py` | Host | Drives the mission via MAVLink |

The container hosts the Rerun stream itself (`rr.serve_grpc`) on port **9877** and the
viewer connects *into* it through the published port. Don't rely on the container dialing
out to `host.docker.internal`: with docker running natively inside WSL, `host-gateway`
resolves to the WSL VM rather than Windows, so a viewer on Windows never receives data.
UDP ports are still used internally:

- Camera frames: ports **6000** (downward) and **6002** (forward)
- Range finder: port **6004**
- Telemetry (position/attitude): port **4000**

## Build

Build the image once:

```bash
docker compose build
```

## Run

terminal 1 — start the container (headless PyBullet + ArduPilot SITL):
```bash
docker compose up
```

terminal 2 — attach the Rerun viewer to the container's stream (start it after the
container; data logged before the viewer connects is buffered, so nothing is lost):
```bash
warg run SITL-Plus rerun
# equivalent to: uv run rerun rerun+http://127.0.0.1:9877/proxy
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
