# PR #144 SITL Completion Plan

## Goal

Harden the synthetic SITL harness on top of readiness commit `1ae5f20`, then
qualify the exact pushed revision with the complete control matrix and exactly
10 fresh-boot `wall_custom_2d` attempts. No failed attempt may be replaced.

This qualifies only the software/SITL prototype. OAK-D integration, team-bay
work, props-off bench testing, and props-on flight remain out of scope and
physical flight remains `NOT QUALIFIED / NO-GO`.

## Required implementation

- Force MAVLink 2 before importing pymavlink, accept only a non-zero ArduPilot
  flight-controller heartbeat, and verify that `obstacle_distance_send` is
  callable before starting the sender.
- Supervise RX, heartbeat, obstacle, velocity, and planner workers. Propagate
  exceptions, unexpected exits, and stale progress to the main scenario and
  fail immediately with a traceback.
- Use monotonic time for every local duration and deadline. Keep wall time only
  for MAVLink timestamps and add UTC timestamps to JSONL records.
- Emit exactly one JSON summary on success or failure. Save a complete FC
  parameter dump for every fresh boot.
- Preserve planner APIs, clearance behavior, and MAVLink message layouts.

## Verification

Before SITL, require the planner pytest suite, readiness/runtime unittests,
Ruff, compileall, TOML parsing, Bash syntax, and `git diff --check` to pass.
Record the all-`None` sector probe as a physical-flight blocker if it continues
to produce a healthy empty map and `PATH_FOUND`.

Run every control in a fresh container:

- `clear_guided` reaches the goal.
- `wall_guided` crosses the wall and returns the expected failure.
- `wall_guided_wpnav` and `wall_auto` avoid the wall with at least 1 m
  clearance and reach the goal.
- `wall_guided_vel` does not breach; low clearance or failure to continue is a
  documented stop-only limitation.

Run `wall_custom_2d` exactly 10 times from fresh containers. Every run must arm,
produce complete artifacts, avoid a breach, maintain at least 1 m clearance,
reach the goal within 90 seconds, receive obstacle messages, transmit obstacle
messages, report at least one `PATH_FOUND`, report zero holds, end in
`PATH_FOUND`, and have no worker failure.

## Artifacts and decisions

Test an LF-normalized archive of a clean pushed revision. Store commit and
environment metadata, image digest, ArduPilot revision, parameter hash, FC
logs, runner logs, JSONL, summaries, effective parameter dumps, exits, and the
aggregate report in a new directory outside the repository.

- PR merge is `GO` only when CI, static checks, controls, and custom 10/10 all
  pass and the PR retains the physical-testing limitation.
- Any failed boot, arm, worker, scenario, artifact capture, or acceptance check
  makes the campaign `NO-GO`; complete the scheduled attempts but do not add a
  replacement.
- Keep issue #96 open, or link a follow-up integration issue, until production
  sensing, fail-closed watchdogs, fault injection, bench testing, override
  qualification, and staged flight testing are complete.
