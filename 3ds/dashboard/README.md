# 3DS read-only drone dashboard

React/TypeScript dashboard served by **`npm run dev`**. It displays behaviour-tree
state, preflight results, ROS topics/messages, MAVROS telemetry, image previews and
companion CPU/GPU temperatures. It sends no flight commands and saves no telemetry
logs. The ROS container supplies data; it does not serve the webpage.

## Start the dashboard

Use Node 22+ and npm. From the repository root:

```bash
cd 3ds/dashboard
npm ci
npm run dev
```

Open **http://127.0.0.1:5173/** for live ROS data. To try the UI without ROS, open
**http://127.0.0.1:5173/?mode=mock&image=%2Fcamera%2Fimage_raw**. Mock mode has a
persistent synthetic-data banner; it uses the same parsers/components as live mode
but opens no ROS connection. The sample image is synthetic.

Vite stays running in this terminal. Ctrl+C stops it. It binds to loopback and
uses port 5173; an occupied port produces an error instead of silently changing
ports. To use a different port:

```bash
npm run dev -- --port 5174
```

Node/npm and source dependencies must be present wherever this server runs.
There is no build/preview command or bundled frontend in the ROS Docker image.

## Connect to ROS

```text
ROS publishers -> rosbridge / rosapi -> WebSocket -> browser
                                        ^
Vite (npm run dev) -> webpage ------------+
```

The frontend's server and ROS environment can run on different computers. In
Connection Settings, enter the reachable `ws://HOST:PORT` of the intended
rosbridge. The default is the webpage's hostname on port 9090 (`wss://` for an
HTTPS page). URL overrides continue to work, for example:

```text
http://127.0.0.1:5173/?bridge=ws%3A%2F%2Flocalhost%3A9091
```

The regular `3ds.launch.py` already starts rosbridge/rosapi, the mission engine,
camera and MAVROS. It also starts companion temperature telemetry. Connect to
its existing bridge; do not launch another on the same port. Do not start the
full mission stack just to work on the webpage.

For an existing ROS environment that needs only a bridge, build/source the ROS
workspace and use the optional monitoring launch:

```bash
cd 3ds
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch airside_bringup dashboard.launch.py
```

This starts rosbridge/rosapi only. It starts no FCU, camera, mission or webpage
server. Match `ROS_DOMAIN_ID` to the intended ROS graph. If needed, enable
`temperature_enabled:=true` only on the intended companion host. Measurements
belong to that host, not the browser computer.

Alternatively, `docker compose -f 3ds/compose.dashboard.yaml up --build` starts
this same bridge-only launch. It requires Docker host networking and does not
start Node/Vite. Run the frontend separately.

For Gazebo/SITL telemetry and actual rendered images, follow
[the simulation setup](../sim/README.md).

## Run on the companion computer

To host the dashboard airside, run the Node commands above **on the companion
host**, alongside its ROS environment. Docker alone will not start the webpage.
For access over an authenticated SSH tunnel from the ground computer:

```bash
ssh -N -L 5173:127.0.0.1:5173 -L 9090:127.0.0.1:9090 USER@COMPANION
```

Then browse http://127.0.0.1:5173/ on the ground computer. The default bridge URL
uses the forwarded port 9090. Substitute the actual ports if they differ.

For direct access on a trusted local network, Vite can bind externally with
`npm run dev -- --host 0.0.0.0`; the ROS bridge must also be reachable at its
configured address. Protect both listeners. Vite serves development source and
rosbridge remains command-capable even though the dashboard is read-only. Do not
expose either service to the public internet. Keep Vite's host checks enabled.

## Configuration

| Setting | Default | Purpose |
|---|---|---|
| `?mode=mock` | absent/live | Clearly labelled synthetic demonstration; no ROS access |
| `?bridge=ws%3A%2F%2Flocalhost%3A9091` | webpage host, port 9090 | Override the WebSocket endpoint; also editable in Connection Settings |
| `?image=/camera/image_raw` | no selection | Initially select a discovered image topic |
| `?preflight=/preflight/diagnostics` | `/preflight/diagnostics` | Dedicated diagnostic snapshot topic |
| Bridge launch `bridge_bind` / `bridge_port` | `127.0.0.1` / `9090` | Standalone ROS listener |
| Bridge launch `bridge_enabled` | true | Disable if a bridge and rosapi already exist |
| Bridge launch `temperature_enabled` | false | Enable sensor collection on this ROS host |
| Temperature node `sys_root` | `/sys` | Sensor filesystem root; tests use temporary fixtures |
| `ROS_DOMAIN_ID` | inherited | DDS discovery domain |

URL-encode query values when constructing links. No endpoint settings are saved
to localStorage. Automatic retry backs off from 1 to 30 seconds; discovery refreshes
every 10 seconds. Missing, invalid, disconnected or >5-second-old samples are
identified explicitly. Receipt age uses the browser clock; source time is ROS time.

## Data sources

| Feature | Topic/service | Type and meaning |
|---|---|---|
| Behaviour tree | `/dashboard/tree` | `std_msgs/msg/String`, JSON v1 observer snapshot every second: node IDs, parents, names, statuses, feedback and active tip |
| Preflight | `/preflight/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray`, dedicated complete results from issue #210 |
| Topics | `/rosapi/topics` | `rosapi/srv/Topics`, discovered names/types |
| FCU state | `/mavros/state` | `mavros_msgs/msg/State`, connection, arming, mode, guided flag and numeric system status |
| GPS | `/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix`, latitude/longitude and WGS84 ellipsoid altitude |
| Attitude | `/mavros/imu/data` | `sensor_msgs/msg/Imu`, quaternion-derived roll/pitch/yaw in the ROS frame |
| Local position | `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped`, metres in the reported ROS frame |
| Images | selected discovered topic | `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage` |
| Temperatures | `/dashboard/temperatures` | `std_msgs/msg/String`, JSON v1 with timestamp, monitoring host/source and CPU/GPU sensor arrays every two seconds |

The engine observer never ticks the mission or changes its state. Historical
snapshots are labelled; collapsing branches only changes the UI. Topic search,
grouping and All Topics operate on discovered metadata without extra ROS requests
or subscriptions. Shared consumers use reference-counted subscriptions.

Issue #210 must publish the actual checks; this dashboard does not implement them.
Use a dedicated periodic **complete snapshot**, including `header.stamp` and every
required `status[]` with `name`, `level`, `message` and `values` (empty array allowed).
Levels 0/1/2/3 mean Passed/Warning/Failed/Unavailable. No message, empty results,
malformed data, stale results or disconnection can establish a pass. A reported
pass covers only the supplied checks; it never replaces autopilot pre-arm checks.
Do not interpret unrelated OK entries on general `/diagnostics` as flight readiness.

Temperature readings come from identified CPU/GPU thermal/hwmon sensor files on
the monitoring host. Missing/unreadable/unsupported sensors give empty arrays,
not substitute temperatures. Browser hardware readings are never used. Physical
companion sensor labels and permissions still need hardware verification.

## Images and message limits

Raw images support rgb8/bgr8/rgba8/bgra8/mono8 with row padding and base64 or
byte-array data; compressed previews support JPEG/PNG, excluding compressedDepth.
Unsupported encodings are reported. A valid black frame is still a received frame;
the existing camera stub can emit black fallback images.

Raw frames are capped at four million pixels and 24 MB payloads. Preview delivery
is throttled to at most two frames/s with queue length one. Image selection starts
no camera. The topic inspector keeps the latest JSON message (16,000 characters)
and a short receipt-time history. Its displayed rate is **browser-received rate**,
not original publisher frequency.

The pinned roslib dispatcher uses `src/rosbridge.ts` to reassemble fragmented
JSON before normal dispatch. It permits eight incomplete groups, 256 parts/group,
16 Mi characters/message, 32 Mi retained characters and a ten-second lifetime.
Disconnect/cleanup clears partial groups. Unsupported protocol compression and
malformed/conflicting fragments produce visible receive errors. This is separate
from JPEG/PNG image decoding; normal telemetry and subscriptions use the same
transport as before.

## Checks

From the repository root:

```bash
bash 3ds/dashboard/check.sh
python3 -m unittest discover -s 3ds/sim/test -v
```

The check script installs locked npm dependencies on a fresh checkout, then runs
TypeScript (`npm run typecheck`), ESLint, frontend tests and ROS-independent sensor/
tree tests. It generates no frontend bundle or validation files. Protocol/server
tests need permission to bind temporary loopback sockets. Run the existing ROS
workspace tests separately with `colcon test` / `colcon test-result --verbose`.

Optional ROS integration tests use their own private containers and labelled
fixtures. They never connect to or fly the running simulator:

```bash
docker build --target prod -f 3ds/docker/Dockerfile -t warg/3ds-dashboard-verify:issue213 .
RUN_ROS_DASHBOARD_INTEGRATION=1 npm --prefix 3ds/dashboard test -- test/ros-integration.test.ts
RUN_MONITORING_INTEGRATION=1 npm --prefix 3ds/dashboard test -- test/monitoring-integration.test.ts
```

These check rosbridge fragmentation, actual engine snapshots without an FCU, and
diagnostic/sensor-file fixtures. They cannot establish physical-drone compatibility,
complete preflight readiness or progression through a real mission.

## Troubleshooting

- Page won't open: keep `npm run dev` running and use its printed URL. If the port
  is occupied, reuse the intended server or choose `--port`; do not start duplicates.
- Bridge disconnected: check Connection Settings, rosbridge and network reachability.
  A remote browser's `localhost` refers to that browser's own computer.
- No topics: check rosapi and `ROS_DOMAIN_ID`. A connected WebSocket alone does not
  prove ROS discovery is working.
- Stale/missing telemetry: check the producer, topic/type and receipt age. A topic
  can exist solely because subscribers exist; that does not prove publishing.
- No camera signal: select the intended image topic and check the camera adapter,
  encoding and fresh frame timestamps. A stationary scene can still be live.
- No temperatures/preflight/tree: verify their producers exist. Missing sources
  should remain unavailable; do not replace them with unlabelled fixtures.
