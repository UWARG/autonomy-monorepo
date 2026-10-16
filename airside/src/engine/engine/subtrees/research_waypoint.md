# Research: Faster Waypoint Following

Status: Initial research and proposed comparison; no performance results yet.

## Objective

Compare waypoint-following approaches and recommend a method to test against the current implementation. The goal is to reduce the time required to complete a valid lap while respecting waypoint order, course requirements, flight boundaries, and aircraft limits.

This research concerns navigation between waypoints. Animal detection is a separate part of the overall mission.

## 1. What the current code does

The current approach is **sequential point-to-point waypoint following using position-only setpoints**.

```text
Load waypoint A -> command flight toward A -> detect arrival
    -> load waypoint B -> command flight toward B -> ... -> repeat
```

The work is distributed across several files:

| File | Responsibility |
| --- | --- |
| [lapping.py](lapping.py) | Builds the behavior tree that repeats waypoint visits and manages the lapping phase. |
| [load_next_waypoint.py](../behaviors/navigation/load_next_waypoint.py) | Selects the next waypoint from the list. |
| [fly_to_waypoint.py](../behaviors/navigation/fly_to_waypoint.py) | Publishes the current destination to `mavros/setpoint_raw/global` and checks arrival. |
| [lap_timing.py](../behaviors/navigation/lap_timing.py) | Resets the waypoint index and records lap timing. |
| [time_checks.py](../behaviors/navigation/time_checks.py) | Checks the deadline and whether another lap is expected to fit. |
| [constants.py](../constants.py) | Currently sets a 1 m arrival radius, a 120 s lapping window, and a 500 ms behavior-tree tick period. |

`FlyToWaypoint` ignores velocity, acceleration, yaw, and yaw-rate fields in its outgoing message. It reports success when the estimated three-dimensional distance to the destination is within the acceptance radius. The tree can then select the next waypoint.

This does **not** explicitly require a full stop at each waypoint. It also does not prove that the actual flight path consists of perfect straight lines: ArduPilot determines the underlying vehicle motion.

Before lapping, [load_waypoint_list.py](../behaviors/navigation/load_waypoint_list.py) calls `sort_clockwise_sweep()` in [waypoint_utils.py](../../../../../utils/src/waypoint_utils.py). This sorts points by their angle around their centroid. It is a simple ordering heuristic, not an exhaustive search or a fastest-lap optimizer. If the competition specifies waypoint order, that prescribed order must take priority.

### Why use this approach initially?

The history includes an initial lapping implementation and an M1 flight-test integration. These suggest an early functional baseline, although the author's exact design rationale is not documented here. See [the initial lapping commit](https://github.com/UWARG/autonomy-monorepo/commit/a628c58f5ba7241acf84499d0f5829e2caf11e98).

Likely engineering reasons are:

- **Simple implementation:** one destination and one arrival condition are easy to inspect.
- **Reuse of the flight controller:** the application can issue goals without implementing low-level flight control.
- **Straightforward mission integration:** timing, repeated laps, and transitions can be implemented independently of a custom trajectory planner.
- **Useful comparison baseline:** more complex methods should demonstrate a measurable improvement over this version.

The main limitation is that this application provides only the current destination, with no outgoing velocity or explicit preview of the next leg. Whether that causes substantial slowing must be established from logs or simulation.

## 2. Candidate approaches and why they matter

These approaches operate at different levels. A **planner** creates a route; a **follower** tracks it. They can be combined rather than treated as mutually exclusive replacements.

### A. Pure Pursuit / lookahead following

Pure Pursuit selects a point ahead on a reference path and calculates a turning command toward it. Lookahead distance trades responsiveness against tracking accuracy. The original formulation is for ground vehicles and does not model actuator dynamics. [Coulter's original technical report](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf).

**Why consider it:** continuous reference-path tracking is worth comparing with discrete destination switching. It is also a relatively accessible starting point for understanding lookahead guidance.

**Limitation:** it does not generate a fastest route or speed profile. A multirotor adaptation needs suitable position or velocity commands and cannot simply use a car's steering equation. Excessive lookahead may miss required waypoint regions.

### B. Spline / Bezier path with a speed profile

Construct connected curves between waypoints, with compatible tangent directions at joins, and assign speeds along the resulting path. This is a family of planning techniques, not one complete controller. ArduCopter already supports spline mission waypoints that produce curved horizontal and vertical paths. [Official spline waypoint documentation](https://ardupilot.org/copter/docs/mission-command-list.html#spline-waypoint).

**Why consider it:** planning across several waypoints allows the outgoing direction to influence the approach to a turn. It gives a custom implementation explicit control over path shape and speed scheduling.

**Limitation:** a smooth curve is not automatically faster or dynamically feasible. It needs a follower, speed/acceleration limits, and checks along the whole curve for boundary and waypoint compliance. Adding many intermediate position targets alone does not establish an effective trajectory-tracking system.

### C. Dubins paths

Dubins paths combine straight segments and circular arcs. They give a shortest path between two specified positions and headings under a planar, constant-forward-speed model with a minimum turning radius. [LaValle, Planning Algorithms: Dubins Curves](https://lavalle.pl/planning/node821.html).

**Why consider it:** this makes turning-radius constraints explicit, making it particularly relevant to fixed-wing cruise.

**Limitation:** the result is not a general minimum-time aircraft trajectory. Wind, changes in altitude, roll transitions, and speed changes require additional treatment. Connecting individually optimal pairs also does not guarantee an optimal full lap, especially when intermediate headings are not specified.

### D. Existing ArduPilot navigation as an additional benchmark

For ArduCopter, AUTO waypoint navigation already anticipates turns using S-Curves and applies acceleration constraints. Guided position targets can also use S-Curve planning when `GUID_OPTIONS` bit 6 is enabled, but this is not suitable for rapidly changing targets. Neither capability is confirmed to be active on the team's aircraft. [AUTO documentation](https://ardupilot.org/copter/docs/auto-mode.html), [Guided documentation](https://ardupilot.org/copter/docs/ac2_guidedmode.html).

**Why consider it:** the current project already communicates with ArduPilot, so its built-in navigation should be compared before investing in a custom replacement. Integrating an AUTO mission still requires mission-progress handling and interruption/resume behavior.

For fixed-wing flight, investigate ArduPlane's existing L1 navigation instead of assuming Copter behavior applies. [ArduPlane navigation documentation](https://ardupilot.org/plane/docs/navigation-tuning.html).

## 3. Initial decision matrix

These are qualitative engineering judgments about implementation effort and suitability, not measured rankings or guaranteed speed improvements.

| Approach | Role | Reason to include | Main limitation | Initial priority |
| --- | --- | --- | --- | --- |
| Current sequential method | Destination sequencing | Establishes actual baseline behavior and lap time | No application-level multi-leg trajectory or speed planning | Required baseline |
| Pure Pursuit / adapted lookahead | Path follower | Accessible way to investigate continuous tracking | Needs a reference path, speed policy, and aircraft-specific command mapping | Research prototype |
| Spline / Bezier + speed profile | Planner, paired with a follower | Allows deliberate turn geometry and speed scheduling | More integration work; smoothness alone does not guarantee speed or compliance | Main custom-planning candidate |
| Dubins + suitable follower | Geometric planner | Explicit minimum turning radius | Simplified aircraft model; requires headings | Prioritize if fixed-wing cruise is confirmed |
| Built-in ArduPilot navigation | Existing navigation stack | Tests whether existing capabilities address the performance gap | Depends on firmware, parameters, aircraft, and mission integration | First practical comparison for the relevant aircraft |

## 4. Proposed recommendation

**Keep the current approach as the baseline, evaluate the applicable built-in ArduPilot navigation, and research a smooth path plus speed profile as the main custom alternative.**

Why this order:

1. A baseline prevents claiming improvements without evidence.
2. Existing navigation may deliver the required behavior with less custom code to maintain.
3. A custom smooth-path approach targets turn geometry and speed scheduling directly, while allowing explicit course constraints.
4. Lookahead following is worth investigating as a tracking component, not automatically as a complete replacement planner.
5. If fixed-wing cruise is confirmed, Dubins planning and L1 tracking become more relevant comparison points.

No final performance winner can be selected from code inspection alone.

## 5. How to validate the recommendation

- Use identical waypoint order, course constraints, aircraft model, speed limits, and acceleration limits for every comparison.
- Include the final-to-first segment when measuring a closed lap. Measure between successive crossings of the same start/finish reference, after the initial approach.
- Record valid completed laps, lap time, closest approach to required waypoints, boundary violations, turn speeds, and tracking error.
- Compare several runs and include wind/disturbance cases. Report median lap time and completion rate; reject apparent speed gains caused by skipping waypoints or shortening the permitted course.
- Verify lap completion independently of the current bookkeeping: the waypoint index advances when a target is loaded, and the tree converts waypoint-loop failures into success. A failed final waypoint must not count as a completed lap.

The current 1 m acceptance radius and 120 s lapping duration are implementation settings, not verified competition requirements. Before implementation, confirm the applicable competition year, required waypoint-passing geometry, aircraft/flight mode, and flight-controller configuration. Those unknowns should remain explicit rather than be guessed.
