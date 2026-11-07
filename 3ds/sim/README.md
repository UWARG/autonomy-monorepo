# Gazebo/SITL dashboard monitoring

Optional adapters for observing an existing ArduPilot/Gazebo simulation. This
setup starts MAVROS, rosbridge and a rendered-camera adapter. It does not start
WARG mission logic or send arming/flight commands.

## Prerequisites

- Native Gazebo Harmonic with the `ardupilot_gazebo` plugin and `iris_runway` world.
- An already-built ArduCopter SITL binary with the JSON model and MAVProxy.
- Native Python Gazebo transport/message bindings and Pillow.
- Docker with host networking enabled, and the dashboard's ROS image.
- Node 22+ with dashboard dependencies installed (`npm ci` in `3ds/dashboard`).

The commands below assume ArduPilot and its Gazebo plugin are installed under
`~/ardupilot` and `~/ardupilot_gazebo`. Adapt those paths to your installation.
Check for existing processes and occupied ports before starting duplicates.
Use separate terminals for long-running commands.

## Container

Build the image from the repository root if needed:

```bash
docker build --target prod -f 3ds/docker/Dockerfile -t warg/3ds-dashboard-verify:issue213 .
docker run -d --name warg-dashboard-sim --network host \
  -v "$PWD:/monorepo" warg/3ds-dashboard-verify:issue213 sleep infinity
```

If this dedicated container already exists, inspect it and reuse it with
`docker start warg-dashboard-sim`. Do not start the hardware/full-mission stack
for this monitoring setup.

## Simulator

Start the server independently so closing the optional viewer does not stop
physics or camera output:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -s -r iris_runway.sdf
```

In another terminal, an optional viewer can connect with `gz sim -g`.

Start the existing SITL binary with the supplied frame defaults and fresh,
temporary simulator state. This does not rebuild firmware or reset hardware:

```bash
SITL_RUN_DIR=$(mktemp -d /tmp/warg-sitl.XXXXXX)
cd "$SITL_RUN_DIR"
"$HOME/ardupilot/build/sitl/bin/arducopter" --model JSON --speedup 1 -I0 \
  --defaults "$HOME/ardupilot/Tools/autotest/default_params/copter.parm,$HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm"
```

Connect MAVProxy from a separate terminal. That terminal is its interactive
command prompt; the Map and Console windows are status displays:

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --console --map
```

Confirm that the intended native SITL owns TCP port 5762 before connecting ROS.
Keep the aircraft disarmed for initial monitoring checks.

## ROS, camera and browser

From the repository root, start the isolated monitor:

```bash
WSL_SITL_IP=$(ip -4 -o addr show dev eth0 | awk '{print $4}' | cut -d/ -f1)
docker exec -it -e ROS_DOMAIN_ID=73 warg-dashboard-sim /entrypoint.sh \
  ros2 launch /monorepo/3ds/sim/monitor.launch.py "fcu_url:=tcp://${WSL_SITL_IP}:5762"
```

This uses domain 73 and bridge port 9091. Domain separation also isolates
MAVROS's absolute `/uas1` route. Recompute the WSL address after a restart.

Once MAVROS reports its FCU connection, request telemetry without persistent
parameter changes:

```bash
docker exec -e ROS_DOMAIN_ID=73 warg-dashboard-sim /entrypoint.sh \
  ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  '{stream_id: 0, message_rate: 2, on_off: true}'
```

Prepare the downward camera and optional visual-only ground patch once per world,
then leave the adapter running:

```bash
python3 3ds/sim/setup_scene.py
# If the patch already exists: python3 3ds/sim/setup_scene.py --camera-only
python3 -u 3ds/sim/gazebo_camera.py --container warg-dashboard-sim
```

The patch has no collision geometry. The adapter reads the actual Gazebo
RGB_INT8 camera, JPEG-encodes it and forwards the latest frame at up to 2 Hz.
It publishes `/gazebo/camera/image/compressed`; it does not generate test footage.

Start the frontend in another terminal:

```bash
npm --prefix 3ds/dashboard run dev -- --port 5174 --strictPort
```

Open [the live simulation dashboard](http://127.0.0.1:5174/?bridge=ws%3A%2F%2Flocalhost%3A9091&image=%2Fgazebo%2Fcamera%2Fimage%2Fcompressed).
Do not use `mode=mock` for simulator telemetry.

## Expected behavior and troubleshooting

- State, GPS, attitude and local position come from MAVROS. GPS altitude uses
  the WGS84 ellipsoid; it is not height above the takeoff point.
- Vertical movement primarily changes local Z and altitude. Latitude/longitude
  and local X/Y need horizontal movement to change appreciably.
- A stationary camera view can still be live: check receipt age and forwarded
  frame counts. No signal requires checking the adapter process and selected topic.
- No GPS/position data: verify the FCU link and repeat the runtime stream request.
- FCU link lost while the browser remains connected: check that Gazebo and SITL
  are still running and physics is not paused.
- Behaviour tree, preflight diagnostics and temperatures are unavailable here
  because this monitor does not start their producers. Do not fill them with
  synthetic values or treat missing diagnostics as a pass.

Run the camera adapter's unit tests without launching a simulator:

```bash
python3 -m unittest discover -s 3ds/sim/test -v
```

To stop, land and disarm first if you have manually flown the simulation. Then
Ctrl+C the processes you started and stop only your dedicated container with
`docker stop warg-dashboard-sim`. No simulator parameters, recordings or PID files
belong in the PR. rosbridge remains command-capable: keep it on a trusted network.
