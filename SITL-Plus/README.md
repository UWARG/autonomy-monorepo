# SITL-Plus

A hardware-in-the-loop simulation environment using PyBullet for physics, ArduPilot SITL for flight control, and optional Rerun visualization.

## Prerequisites

- **Windows 11** with WSL 2
- **Mac / older Windows**: install an X Server (e.g. [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [XQuartz](https://www.xquartz.org/)) for the GUI profile
- **Linux**: works out of the box with the built-in Wayland/X11 display server
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) with WSL 2 backend enabled

## Architecture

The simulation is split across processes:

| Process | Where it runs | What it does |
|---|---|---|
| PyBullet sim (`main.py`) | Docker container | Physics engine, camera & range-finder sensor simulation |
| ArduPilot SITL (`sim_vehicle.py`) | Docker container | Flight controller, MAVLink on TCP port 5761 |
| Airside (`rerun_airside.py` or `airside.py`) | Host | Receives sensor data from container, drives the mission via MAVLink |
| Groundside (`rerun_groundside.py` or `groundside.py`) | Host | Receives forwarded range-finder data |

Sensor data flows over UDP from the container to the host via `host.docker.internal`:
- Camera frames: ports **6000** (downward) and **6002** (forward)
- Range finder: port **6004** (forwarded to host port **6005** for groundside)
- Telemetry (position/attitude): port **4000**

## Build

Build the image once — both profiles share it:

```bash
docker compose --profile rerun --profile gui build
```

## Run

Two modes are available. Pick one per session.

### Rerun mode (headless sim + Rerun visualizer)

terminal 1 — start the Rerun viewer (groundside):
```bash
uv run rerun
```

terminal 2 — start the container (headless PyBullet + ArduPilot SITL, no GUI):
```bash
docker compose --profile rerun up
```

terminal 3 — airside mission controller + sensor relay + Rerun logging:
```bash
warg run sitl-plus rerun_airside
```

### GUI mode (PyBullet GUI + MAVProxy map/console)

Requires an X Server on Windows/Mac, or a native display on Linux.

terminal 1 — start the container (PyBullet GUI + MAVProxy with map and console):
```bash
docker compose --profile gui up
```

terminal 2 — airside mission controller + sensor relay:
```bash
warg run sitl-plus airside
```

terminal 3 — groundside range-finder + camera viewer:
```bash
warg run sitl-plus groundside
```

## Local SITL (no Docker)

Follow the ArduPilot [Linux build guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux). Once set up, run this from the `ardupilot` directory in WSL:

```bash
python3 ./Tools/autotest/sim_vehicle.py -N -v ArduCopter -f quad \
  --model JSON:<YOUR_IPV4_ADDR> --console --map \
  --out tcpin:0.0.0.0:5761
```

Replace `<YOUR_IPV4_ADDR>` with your WSL host's IPv4 address. Also set `SENSOR_HOST` to `127.0.0.1` in your environment when running airside/groundside scripts locally.

## Logs

Container logs are written to `./logs/` (mounted into the container):
- `logs/pybullet.log` — PyBullet / `main.py` output
- `logs/sim_vehicle.log` — ArduPilot SITL / MAVProxy output
