# Perception Module

Computer vision and target localization algorithms.

## `target_location`

Converts a target's pixel location in a rectified camera image into a 3D position
relative to the drone.

```python
from target_location import Attitude, CameraIntrinsics, ImageFrame, locate_target_frd

position = locate_target_frd(
    image_frame=ImageFrame(u=412.0, v=190.0),
    intrinsics=CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0),
    distance_to_plane_m=12.5,
    attitude=Attitude.from_euler(roll_rad=0.05, pitch_rad=-0.10),
)
# -> TargetPosition(forward_m=..., right_m=..., down_m=...), or None if not localizable
```

It back-projects the detection into a ray, rotates it into the drone's body frame using
the camera's mounting transform, builds the ground plane from the drone's attitude and
the measured distance, and intersects the two.

`ImageFrame` is the detection's location in the image's coordinate frame — this module
never touches pixel data. A bare `(u, v)` pair is accepted anywhere an `ImageFrame` is.

### Orientation input

`Attitude` reduces the drone's orientation to the only part that affects the answer:
which way is down in the body frame. Heading drops out — the result is in the body
frame, and a level plane looks identical from every heading.

| Constructor | Use when your source gives you |
| --- | --- |
| `Attitude.from_euler(roll_rad, pitch_rad, yaw_rad=0.0)` | Tait-Bryan angles in NED. Yaw is accepted for call-site convenience and ignored. |
| `Attitude.from_quaternion(w, x, y, z)` | A quaternion rotating **body-FRD → NED**. Immune to gimbal lock. |
| `Attitude.from_mavros_quaternion(w, x, y, z)` | A `sensor_msgs/Imu` orientation off `/mavros/imu/data`. |
| `Attitude.level()` | Nothing — a perfectly level drone, mostly for tests. |

> [!WARNING]
> MAVROS follows the ROS convention and publishes **body-FLU → ENU**, not FRD → NED.
> Passing a raw `/mavros/imu/data` quaternion to `from_quaternion` silently flips the
> forward axis, placing targets behind the drone when they are in front of it. Use
> `from_mavros_quaternion` for anything coming off that topic.

### Output

`TargetPosition(forward_m, right_m, down_m)`, with `.to_array()` and `.range_m`.

This is a **position, not a pose**. A single ray to a detection centroid carries no
information about how the target is *oriented*; returning an orientation would mean
fabricating three numbers. If you need target heading, it has to come from the detector
(e.g. an oriented bounding box), not from this geometry.

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
