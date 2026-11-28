# Survey Executive

Runs a whole 3DS survey, from a boundary polygon to a covered map
([#278](https://github.com/UWARG/autonomy-monorepo/issues/278)). It plans
the area with gnc's `plan_swarm_orthomosaic`, flies each drone's route in
order, tracks which ground the photos actually cover, reshoots the gaps, and
hands a dropped drone's unflown setpoints to the drones still flying.

## Usage

```python
from src.image_stitcher.stitcher import ImageStitcher
from src.survey_executive import SurveyExecutive, SurveyRequest, SurveyRunner

request = SurveyRequest(
    survey_id="field-a",
    boundary=boundary,          # list[Coordinate], at least 3 points
    camera=camera_spec,         # gnc CameraSpec
    target_gsd_m=0.01,
    max_reshoot_rounds=2,
)
stitcher = ImageStitcher()
executive = SurveyExecutive(
    request,
    manager,                    # the drone state manager (#153)
    stitcher_rejections=lambda: stitcher.rejections,
)
executive.start()               # validates, plans, starts sending setpoints
runner = SurveyRunner(executive)
runner.start()                  # calls executive.tick() twice a second

# Whatever receives photos gives each one to both:
stitcher.add(message)
executive.on_photo(message)     # safe from any thread

executive.progress().to_dict()  # for the dashboard (#213)
executive.abort()               # operator abort; no more commands are sent
runner.stop()
```

- `start()` uses the drones the manager reports as connected. Bad input
  (boundary, camera, GSD, overlaps, no drones) raises before any command is
  sent, and the survey goes to `ABORTED` with the reason.
- `manager` is anything with `get_all_drones()` and
  `send_location_command_sync(...)` (`DroneManagerLike`), so the unit tests
  run on a fake fleet.
- Feeding photos to the stitcher stays with the caller. The executive only
  reads `stitcher.rejections`, and matches them to photos by
  `(drone_id, image_id)`. Photos sent before `start()` are dropped, and a
  photo sent twice counts once.
- `tick()` sends the drone commands while holding the executive's lock, so
  `progress()` and `abort()` wait for it. Keep the manager's
  `send_location_command_sync` from blocking for long.

## States

```text
PLANNING -> SURVEYING -> RESHOOTING -> ... -> COMPLETE
    \            \             \
     +------------+-------------+--> ABORTED (bad input or operator abort)
```

- When every setpoint of a round is flown, the executive waits `settle_s`
  for late photos and stitcher rejections, then finds the gaps larger than
  `min_gap_area_m2`.
  - No gaps: `COMPLETE` with `fully_covered` true.
  - Gaps and reshoot rounds left: it plans the gaps with the orthomosaic
    planner and goes to `RESHOOTING`.
  - Gaps and no rounds left (or the planner refuses them): `COMPLETE` with
    `fully_covered` false and the gaps counted in progress.
- If no drone is left, the survey waits until one comes back or the operator
  aborts.

## How it works

| Module | Does |
|---|---|
| `request.py` | `SurveyRequest` and `validate_request` |
| `planning.py` | Planner setpoints to ordered `Mission`s with stable `task_id`s (`<survey>-r<round>-<drone>-<index>`) and a yaw along the pass |
| `footprint.py` | Photo footprint on flat ground from position, roll, pitch, yaw and camera |
| `coverage.py` | `CoverageMap`: union of accepted footprints, rejected photos excluded, gaps |
| `reshoot.py` | Plans every gap with the orthomosaic planner for the next round |
| `dispatch.py` | `RouteDispatcher`: per-drone queues and dropout handling |
| `redistribute.py` | Splits a dropped drone's setpoints among the drones still flying |
| `executive.py` | `SurveyExecutive` (state machine) and `SurveyRunner` (thread) |
| `progress.py` | `SurveyState` and `SurveyProgress` |

**Dispatch.** The executive keeps its own queue per drone instead of going
through `DroneTaskScheduler`, which gives each target to the nearest free
drone and would scramble the planned routes. A drone gets its next setpoint
only after it reports `ARRIVED` with the previous `task_id`.
`send_location_command_sync` only says whether a command was sent, so a
command the drone never reports (for example one it turned down while
`MOVING`) is resent unchanged after `ack_timeout_s`.

**Dropout.** A drone is dropped when it sends no new state update for
`dropout_after_s`. Its unflown setpoints, including the one in flight, are
cut into one in-order chunk per remaining drone, and each chunk goes to the
drone nearest its start. Chunks are never reversed, since each yaw faces
the planned direction of flight. A dropped drone that reports again rejoins
with an empty queue.

**Frames and units.** All coverage maths is in metres east/north of the
boundary's anchor (`utils/src/coordinates.py`). `ImageMessage` angles are
degrees; `Mission.yaw` is radians clockwise from north. The camera is
assumed to point straight down with the image top toward the nose.

## Settings

| Setting | Default | Where |
|---|---|---|
| `forward_overlap`, `side_overlap` | 0.75, 0.65 | `SurveyRequest` |
| `min_gap_area_m2` | 1.0 | `SurveyRequest` |
| `max_gsd_ratio` (photos coarser than this times the target GSD don't count) | 1.5 | `SurveyRequest` |
| `max_reshoot_rounds` | 2 | `SurveyRequest` |
| `settle_s` | 5.0 | `SurveyExecutive` |
| `ack_timeout_s` | 3.0 | `SurveyExecutive` |
| `dropout_after_s` | 10.0 | `SurveyExecutive` |

## Tests

Unit tests are in `tests/unit/survey_executive/`. They run the executive
against a fake fleet with synthetic level photos: a nominal survey, lost and
rejected photos reshot, a drone dropped partway, round limits, abort from
each state, and random dropouts over fixed seeds where every setpoint must
finish exactly once.

```bash
cd 3ds_groundside
uv run --extra dev pytest tests/unit/survey_executive
```
