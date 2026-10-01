# Issue 160: AEAC traffic to BendyRuler

## Scope

This integration gives the Airside lapping behavior a competition-traffic
obstacle source. It does not add physical-object perception. The planner still
flies ArduPilot in `GUIDED`; `ObstacleAwareFlyToWaypoint` publishes an internal
ENU velocity target and the position controller remains the only MAVROS
setpoint publisher.

The obstacle source is explicit:

- `traffic` (deployment default) consumes `/aeac/traffic`.
- `scan` retains the synthetic `LaserScan` regression path.

The behavior subscribes to only the selected source. It never falls back to a
direct waypoint or silently switches sources.

## Data path

```text
AEAC WebSocket (one bidirectional connection)
  |- 1 Hz own-aircraft telemetry while armed
  `- complete traffic events
       -> aeac_bridge /aeac/traffic (TrafficSnapshot)
       -> WGS84 to current local ENU
       -> altitude relevance filter
       -> receipt-age extrapolation + 3 s swept circles
       -> BendyRuler2D
       -> /position_controller/velocity_target
       -> position_controller lease/ownership gate
       -> /mavros/setpoint_velocity/cmd_vel
```

Fresh, valid empty traffic is a clear map. No first snapshot, disconnect,
protocol error, an invalid aircraft, an unverified self identity, or data older
than 2.5 seconds is unhealthy. In those cases navigation holds with zero
velocity and its 120-second active-navigation clock is paused. Recovery resets
planner hysteresis before movement resumes.

For a relevant aircraft, true-north clockwise heading is converted as:

```text
east_velocity  = speed * sin(heading)
north_velocity = speed * cos(heading)
```

Its reported horizontal keep-away is the circle radius. The planner's existing
1 m clearance margin is additional. The adapter does not climb or descend as
an avoidance strategy.

## ROS messages

`airside_interfaces/TrafficAircraft` carries the server identity, WGS84
position, AGL altitude, velocity, heading, and horizontal/vertical keep-away.

`airside_interfaces/TrafficSnapshot` carries receipt time, sequence,
connection/health state, reason, configured own-aircraft index, and the full
aircraft list. Receipt freshness is measured with the subscriber's monotonic
clock; the ROS header is observational only. Repeated stationary snapshots are
fresh updates and are not treated as frozen data.

## Deployment configuration

The following environment variables are required for `traffic` deployment:

- `OBSTACLE_SOURCE=traffic`
- `AEAC_URL`
- `AEAC_TOKEN`
- `AEAC_UAV_ID`
- `AEAC_OWN_AIRCRAFT_INDEX`
- `AEAC_PROTOCOL_VERIFIED=true`

The token is used only to construct the authenticated socket URL and is never
logged. `AEAC_PROTOCOL_VERIFIED` must remain false until the gate below is
completed. Missing or invalid configuration continuously publishes unhealthy
traffic and therefore cannot command lapping motion.

## Protocol verification gate

The repository includes a synthetic fixture for deterministic CI only. It is
not evidence of the live AEAC contract.

Before formal qualification, capture one traffic event from the AEAC test
server, remove credentials and unrelated personal data, and create a metadata
JSON object with these reviewed facts:

```json
{
  "source": "AEAC test server",
  "sanitized": true,
  "captured_at_utc": "2026-01-01T00:00:00Z",
  "snapshot_semantics": "complete",
  "single_bidirectional_connection_verified": true,
  "server_timestamp_semantics": "document the field or its absence",
  "heartbeat_semantics": "document observed behavior",
  "disconnect_semantics": "document observed behavior",
  "server_includes_own_aircraft": true,
  "own_aircraft_identity": {
    "uavId": "verified ID",
    "aircraftIndex": 0
  }
}
```

The formal traffic runner refuses to start without both external files and
stores their SHA-256 verification record with the artifacts:

```bash
AEAC_PROTOCOL_FIXTURE=/secure/path/traffic-event.sanitized.json \
AEAC_PROTOCOL_METADATA=/secure/path/traffic-protocol-metadata.json \
ARTIFACT_DIR=/var/snap/docker/common/issue160-formal-<sha> \
AIRSIDE_IMAGE=warg/airside:<sha> \
./airside/scripts/avoidance/run_airside_traffic_campaign.sh
```

Use a new empty artifact directory and a single uninterrupted WSL process. The
runner always executes seven controls followed by exactly ten fresh-container
moving-traffic attempts; attempts cannot be replaced.

If the captured schema or semantics differs from the current parser, change
and review the interface before setting the verification flag. Do not infer
missing fields.

## Development evidence and current decision

The synthetic development runs demonstrate the implemented safety behavior,
but they are not formal qualification:

- Static exclusion cylinder: reached the goal, no breach, 1.731 m minimum
  clearance beyond keep-away, zero holds, final `PATH_FOUND`.
- Dropout, malformed traffic, reconnect, and pilot takeover: each stopped,
  recovered on fresh input, reached the goal, and had no breach.
- Single moving crossing: reached the goal in 24.171 seconds with no breach and
  4.342 m minimum clearance, but recorded 14 holds.

That moving run fails the strict nominal requirement `hold_count == 0`.
Additionally, no sanitized live-server fixture or verified competition
self-identity has been supplied. Therefore the current decision is:

- Software merge: **NO-GO pending protocol evidence and strict 10/10**.
- Competition virtual-traffic flight: **NOT QUALIFIED / NO-GO**.
- Physical-object avoidance: out of scope; there is no OAK-D/depth adapter in
  this integration.

Before flight, complete the live test-server gate, props-off dropout/reconnect/
takeover tests, and a staged low-speed virtual-intruder flight in open airspace
with the team's safety pilot and flight approval.
