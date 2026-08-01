# Perception Module

Computer vision and target localization algorithms.

## `target_location`

Converts a target's pixel location in a rectified camera image into a 3D position
relative to the drone.

```python
from target_location import CameraIntrinsics, locate_target_frd

intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
position = locate_target_frd(
    pixel=(412.0, 190.0),
    intrinsics=intrinsics,
    distance_to_plane_m=12.5,
    roll_rad=0.05,
    pitch_rad=-0.10,
)
# -> array([forward, right, down]) in metres, or None if not localizable
```

It back-projects the pixel into a ray, rotates it into the drone's body frame using the
camera's mounting transform, builds the ground plane from the drone's attitude and the
measured distance, and intersects the two.

### Frames

| Frame | Convention |
| --- | --- |
| Camera optical | OpenCV: `+X` = image right (`+u`), `+Y` = image down (`+v`), `+Z` = optical axis out of the lens |
| Body (FRD) | `+X` forward, `+Y` right, `+Z` down — all outputs are here, in metres |
| World (NED) | Used only to define which way gravity points |

`CameraMount` carries the camera→FRD rotation and the lever arm from the body origin.
`NADIR_DOWN_MOUNT` is the default: camera looking straight down, image right to body
right, image down to body aft.

### Assumptions

- **The image is rectified.** Distortion has already been removed, so no distortion
  coefficients are taken. The intrinsics must be those of the *rectified* image
  (OpenCV's `P` matrix / `camera_info`), not the raw sensor's. If only a field-of-view
  spec is available, use `CameraIntrinsics.from_fov` — it assumes a centred principal
  point.
- **`distance_to_plane_m` is a perpendicular distance**, not a raw rangefinder reading.
  A downward rangefinder on a drone pitched by θ reads `distance / cos(θ)`.
- **The target sits on a locally flat, level plane**, so its normal is world-down. Yaw
  is therefore irrelevant and is not an input.
- **All inputs describe the same instant.** Synchronizing the detection, the range
  measurement and the attitude is the caller's job.

### Failure modes

Malformed inputs — bad intrinsics, a non-positive distance, a non-finite pixel, a mount
that is not a rigid transform — raise `ValueError`; these are caller bugs.

Detections that simply cannot be localized return `None` rather than raising, so a
detection loop is not interrupted: rays that graze the plane (past
`max_incidence_angle_rad`, 85° by default), rays pointing away from the plane, and
intersections beyond an optional `max_range_m`. The threshold exists because position
error grows quadratically with obliquity — a one-pixel detection error displaces the
result by roughly `s² / (f · h)` metres, for slant range `s` and perpendicular distance
`h`.

## Development

```bash
uv sync
uv run --extra dev pytest
uv run --extra dev ruff check .
```
