# Static imaginary-obstacle flight test

## Scope

This configuration is only for a controlled flight test of the Airside
behavior tree and `BendyRuler2D`. It does not connect to the AEAC server and
does not claim competition traffic qualification.

The test publishes the same static obstacle data used by the SITL scenario:

- 20 m north and 0 m east of the first valid armed GPS position
- 15 m AGL
- 5 m horizontal keep-away
- 5 m vertical keep-away
- zero speed

The planner adds its existing 1 m clearance margin. The lapping behavior sends
ENU velocity targets to the position controller, which remains the only MAVROS
setpoint publisher.

## Launch

The normal Airside launch does not start imaginary traffic. Use the explicit
flight-test override:

```bash
cd airside
docker compose -f compose.yaml \
  -f experiments/obstacle_avoidance/compose.flight-test.yaml up --build
```

For SITL:

```bash
docker compose -f compose.sitl.yaml \
  -f experiments/obstacle_avoidance/compose.flight-test.yaml up --build
```

The flight-test launch prints `SYNTHETIC STATIC TRAFFIC ACTIVE`. If that text is
not present, do not proceed with the test.

Defaults can be changed through these environment variables:

```text
STATIC_OBSTACLE_EAST_M=0.0
STATIC_OBSTACLE_NORTH_M=20.0
STATIC_OBSTACLE_ALTITUDE_M=15.0
STATIC_OBSTACLE_KEEP_AWAY_M=5.0
STATIC_OBSTACLE_VERTICAL_M=5.0
FLIGHT_TEST_HORIZONTAL_SPEED_MPS=1.0
```

The waypoint file must place the lapping path through or near the imaginary
obstacle. The synthetic publisher anchors its coordinate at the first valid
armed GPS position, so the pilot must not arm until the aircraft is at the
agreed launch point.

## Readiness gate

Before going to the field, run:

```bash
ARTIFACT_DIR=/var/snap/docker/common/static-obstacle-readiness-<sha> \
AIRSIDE_IMAGE=warg/airside:<sha> \
./airside/experiments/obstacle_avoidance/run_static_obstacle_flight_readiness.sh
```

The gate consists only of what this flight test needs:

- fresh empty traffic reaches the waypoint;
- pilot takeover stops commands and GUIDED resume waits for fresh data;
- three fresh-container static-obstacle runs all reach the goal within 90 s;
- no exclusion breach, at least 1 m clearance beyond keep-away, zero planner
  holds, no global setpoint during lapping, and one MAVROS setpoint owner.

No live AEAC fixture, server identity mapping, moving-traffic campaign, OAK-D,
or physical-obstacle perception is required for this test.

## Flight-day checks

1. Confirm team flight approval, safety pilot, geofence, and open test area.
2. Run the launch props-off and confirm healthy `/aeac/traffic` plus zero
   velocity before arming/GUIDED.
3. Confirm the configured waypoint path and imaginary obstacle location on the
   map before flight.
4. Use the 1 m/s default for the first flight.
5. Verify LOITER takeover stops autonomous commands before attempting the
   obstacle run.
6. Save the ROS bag, diagnostics, MAVROS log, and FC log for comparison with
   SITL.

Passing this test means the synthetic static-obstacle behavior is ready for
comparison between SITL and the real vehicle. The real AEAC connection remains
a separate follow-up.
