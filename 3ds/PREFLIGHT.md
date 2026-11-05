# Preflight diagnostics (issue #210)

The `engine` package's `preflight` executable starts with the normal 3DS launch.
It uses ROS 2 Humble `diagnostic_updater` to publish `diagnostic_msgs/DiagnosticArray`
on `/diagnostics` once per second. It reports readiness; it does not arm the
aircraft, change flight modes, or enforce an arming interlock.

## Checks implemented

| Check | Evidence required | Failure behavior |
| --- | --- | --- |
| Groundstation link | Incoming `/groundstation/heartbeat` (`std_msgs/msg/Empty`) within 3 s | Missing/stale is not ready |
| Camera | `/camera/image_raw` received within 2 s; capture timestamp within 2 s (at most 0.5 s in the future); supported, nonempty image with sufficient brightness and spatial variation | Missing, black, uniform, malformed, unsupported, or stale capture is not ready |
| Expected streams | `/heartbeat`, `/mavros/global_position/global`, `/mavros/global_position/rel_alt` received within 3 s; minimum rates 0.5, 1, 1 Hz | Unknown, stopped, or slow streams are not ready |
| FCU connection | Fresh `/mavros/state` with `connected=true` | Missing/stale/disconnected is not ready |
| ArduPilot pre-arm | Fresh `/mavros/sys_status` with bit 28 present, enabled, and healthy; vehicle disarmed | Unavailable, disabled, failing, or already armed is not ready |
| Photo capture | `/camera/capture` service discovered | Unavailable is not ready |

Images are inspected at up to 2 Hz using a spatial pixel sample. Supported
encodings are `mono8`, `rgb8`, `bgr8`, `rgba8`, and `bgra8`. Row padding and alpha
are excluded. Default mean brightness must be at least 5 and mean per-channel
spatial standard deviation at least 2 (8-bit scale). Tune for the actual camera.
Uniform legitimate scenes can fail these heuristics; textured recorded images
can pass. This is not a proof of camera correctness or a frozen-image detector.

Stream rates use up to five seconds of receive history; at least two samples
are needed. Event-only topics (`/mission/setpoint`, `/mission/go_home`, and
`/position_controller/target`) are deliberately excluded: they can be silent
before takeoff. Topic checks establish delivery, not validity of position values.
Service discovery establishes availability, not successful photo persistence.

ArduPilot owns battery, RC, calibration, GPS/EKF, and other native safety checks.
We consume its explicit `MAV_SYS_STATUS_PREARM_CHECK` bit through MAVROS's
`SysStatus.sensors_present`, `sensors_enabled`, and `sensors_health` fields.
An absence of `PreArm:` text messages is never treated as success. Check
`/mavros/statustext/recv` for the FCU's failure details. The bit is always healthy
while armed, so an armed state explicitly prevents a green preflight result.
This reports the FCU's configured checks; it cannot prove all recommended checks
are enabled in the FCU parameters.

## Groundstation contract

The existing `3DS_Groundside` project is a skeleton. A groundstation client must:

1. Connect to the aircraft's rosbridge WebSocket on port 9090.
2. Subscribe to `/heartbeat` and `/diagnostics`.
3. While receiving fresh `/heartbeat`, send `/groundstation/heartbeat` at 1 Hz.
   Stop sending it when aircraft messages stop arriving. This ties the inbound
   heartbeat to evidence that the outbound link works.
4. Read the diagnostic named `preflight_node: readiness`. Only level `0` with
   key `ready` equal to string `"true"` means ready. Treat WARN (1), ERROR (2),
   STALE (3), unknown, and missing as not ready. Each component has its own status.
5. Use a local receive-time watchdog: clear ready if no readiness update arrives
   for 3 s, including on socket close/reconnect. Never retain a green indication
   across reconnects. Updater startup statuses contain no `ready=true` value.

Example rosbridge messages (send the publish repeatedly, not just once):

```json
{"op":"subscribe","topic":"/heartbeat","type":"std_msgs/msg/Empty"}
{"op":"subscribe","topic":"/diagnostics","type":"diagnostic_msgs/msg/DiagnosticArray"}
{"op":"advertise","topic":"/groundstation/heartbeat","type":"std_msgs/msg/Empty"}
{"op":"publish","topic":"/groundstation/heartbeat","msg":{}}
```

Heartbeat inputs are not latched. Receive timeouts use a monotonic clock, and
diagnostic publication uses a steady timer even if simulation time pauses.
Image capture timestamps use the node's ROS clock and must share its time base.

## Parameters and remapping

Parameters are read at startup. Run independently for tuning (stop the launched
preflight node first to avoid two readiness publishers):

```bash
ros2 run engine preflight --ros-args \
  -p camera_min_mean:=5.0 -p camera_min_std:=2.0 \
  -p groundstation_timeout_s:=3.0 -p camera_timeout_s:=2.0 -p fcu_timeout_s:=3.0
```

`topic_timeout_s` sets the receive timeout for the three expected streams (3 s by
default). Their topic names, message types, and minimum rates are in the short
`topics` list in `preflight_node.py`; add another entry there if needed.
Use ROS remapping for topic names. Invalid thresholds fail startup.

## Validation

Hardware-free decision tests, also wired into the project's `test` command:

```bash
warg run 3ds test
```

To run the actual ROS integration test, start Docker Desktop and use this test-only
setup. It has no camera/serial device mappings and sends fake messages; it never
starts MAVROS or connects to an aircraft:

```bash
cd 3ds
docker compose -f compose.test.yaml run --build --rm preflight-test
```

Expected result: **29 passed**, with no skipped ROS test. The first run downloads
and builds the ROS image, so it takes longer. A successful hardware-free test
confirms the decision rules; this Docker test also confirms ROS subscriptions,
diagnostics publication, and recovery when fake inputs change.

Before flight, verify on SITL/bench: startup stays not ready; send the ground
heartbeat and all healthy streams and confirm readiness; then independently
stop heartbeat/camera/GPS, publish black/malformed/stale camera frames, disconnect
the FCU, cause an FCU pre-arm failure, and remove the capture service. Each must
clear readiness, and recovery must require fresh healthy evidence. Kill the
preflight node and verify the groundstation watchdog clears ready within 3 s.
Do not arm motors as part of the test. Full ROS, rosbridge, and vehicle testing
is required in addition to the hardware-free unit tests.

## Further requirements to confirm with the team

- Photo storage free space and write permissions at the actual output path.
- Mission manager and position-controller liveness beyond ROS graph presence.
- Camera calibration, image exposure thresholds, and capture success on hardware.
- Expected navigation stream rates and validity for the deployed FCU.
- Groundstation mission configuration and any required mission acknowledgement.

These need explicit interfaces/thresholds before adding them to readiness.

References: [issue #210](https://github.com/UWARG/autonomy-monorepo/issues/210),
[ArduPilot pre-arm checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html),
[ROS 2 diagnostics](https://github.com/ros/diagnostics/tree/ros2-humble),
[MAVLink pre-arm bit](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_PREARM_CHECK).
