# Live AEAC backup on PR #181

This is separate from the synthetic static-obstacle flight test. The explicit
launch starts MAVROS, `comms/traffic_listener`, `comms/aeac_telemetry_sender`,
and the visibility-graph A* controller. It excludes the ordinary engine manager
and position controller, so only one node publishes MAVROS setpoints. The
existing `/aeac/traffic` fixture and `TrafficSnapshot` planner remain
synthetic-only.

## CONOPS and identities

The 2027 AEAC CONOPS v1.0 (2026-09-08), page 20, requires 1 Hz telemetry
whenever armed: a bidder-defined string UAV ID, Unix timestamp, decimal-degree
latitude and longitude, altitude AGL in metres, horizontal and vertical
position accuracy in metres, battery percentage 0–100, one of six specified
flight modes, and 0–1 RC and telemetry link statuses. The same page says the
server provides aircraft positions and horizontal/vertical exclusion-cylinder
keep-away distances at 1 Hz. Page 22 lists penalties for telemetry intervals
outside 0.4–1.1 s. Pages 11 and 19 require flight-boundary safety and give a
100 m AGL ceiling.

`AEAC_UAV_ID` is our chosen outgoing string, **not** the token.
`AEAC_CONNECTION_TOKEN` is the team secret for both WebSocket connections.
`AEAC_OWN_AIRCRAFT_INDEX` is the numeric traffic index used to remove our own
aircraft. The CONOPS does not define its mapping, so verify it against the
server before live flight; do not infer it from a name or UAV ID. Missing or
invalid values fail closed. The CONOPS gives required fields but not exact
WebSocket JSON. The `{"action":"telemetry","data":...}` envelope and inbound
`traffic` parser are from the existing AEAC work; confirm them with real
`telemetry_ack` and server traffic before flight. Server documentation is
linked from `aeac.mylonics.com`.

## Configuration and launch

Put the full team token, chosen UAV ID and verified numeric aircraft index in
`airside/.env` on the flight computer. The file is Git-ignored. Never put the
token in a tracked file, shell command, screenshot or ROS bag. Blank token/ID
or index `-1` is deliberately not flight-ready. From `airside/`, choose one:

```bash
docker compose -f compose.yaml -f compose.aeac-backup.yaml up --build
docker compose -f compose.sitl.yaml -f compose.aeac-backup.yaml up --build
```

Never combine this with the synthetic `compose.flight-test.yaml` override.
Live traffic uses `/position_controller/obstacle` plus the atomic
`/position_controller/obstacle_snapshot` marker, not `/aeac/traffic`. Goals
must arrive on `/position_controller/target` at least once per second. This
isolated launch has no mission-goal publisher: the flight-test goal source
must be supplied and checked against the site's flight boundary.

## Props-off go/no-go

This backup is **not** cleared for physical flight merely by image or unit
tests. With props removed and the safety pilot present:

1. Independently verify the site-specific geofence, 100 m AGL limit, flight
   termination, RSO approval and pilot takeover. The backup controller does
   not implement these CONOPS requirements.
2. Verify GPS, `rel_alt` versus site AGL, battery percentage, nonzero GPS
   covariance, FCU state and the first-flight 1 m/s *observed* limit. In the
   ArduCopter 4.5 SITL qualification, `WPNAV_SPEED=80` cm/s leaves headroom
   for the measured speed to remain below 1 m/s. Confirm the actual FCU
   parameter and observed speed before flight; a commanded 1 m/s did not
   consistently keep measured speed below 1 m/s in SITL.
3. Verify both WebSockets connect concurrently with the team token; sender
   diagnostics show `telemetry_ack` and 0.4–1.1 s send intervals; listener
   status shows complete healthy snapshots at about 1 Hz. Cross-check all
   telemetry fields against GCS/server values. The sender currently reports
   both link statuses as `1.0` estimates, **not measured link quality**.
4. Confirm our own `aircraftIndex` with server evidence and its absence from
   obstacle snapshots. Show that empty, invalid, disconnected or >2.5 s old
   snapshots behave correctly; an empty healthy snapshot may allow travel,
   while unhealthy/stale snapshots must hold. Pilot takeover must stop
   setpoints and GUIDED resume must require a new target.
5. Confirm exactly one publisher on `/mavros/setpoint_raw/global` and bag
   traffic, snapshot status, diagnostics, MAVROS position/state, target and
   setpoint. Do not record secrets or full authenticated WebSocket URLs.

If the server schema, token, own identity, AGL datum, speed, geofence,
termination or props-off checks are unverified, leave this live backup off.
