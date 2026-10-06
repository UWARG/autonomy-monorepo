# IMS rosbag recording

The recording widget and ROS controller implement issue #194. The widget appears
in the second row of the IMS dashboard alongside Connection and Mission Script.

## Run the controller

The controller belongs to the existing `airside` ROS 2 Humble `engine` package
and is launched by `engine.launch.py` alongside rosbridge on port 9090. Rebuild
the airside image after adding it; a previously built image does not contain
the new executable or its dependencies.

For a standalone controller in an already built, sourced ROS workspace:

```bash
ros2 run engine rosbag_controller
```

By default it records all discoverable topics to unique directories under
`/ros_ws/data/rosbags`. Both airside Compose configurations persist this directory
under `airside/data/rosbags` on the host. Each recording has a sibling `.log`
containing recorder diagnostics. Configure the node without changing the widget:

```bash
ros2 run engine rosbag_controller --ros-args \
  -p output_directory:=/path/to/bags \
  -p 'topics:=[/heartbeat, /camera/image_raw]'
```

## ROS interface

| Name | Type | Purpose |
| --- | --- | --- |
| `/ims/rosbag/set_recording` | `std_srvs/srv/SetBool` | `data: true` starts; `false` stops. Requests are idempotent. |
| `/ims/rosbag/get_status` | `std_srvs/srv/Trigger` | Returns JSON status in `message`. |
| `/ims/rosbag/status` | `std_msgs/msg/String` | Publishes the same JSON every 0.5 seconds. |

Status contains `recording` (boolean), `bag_path` (string or null), and `error`
(string or null). A successful Start requires a live process and an opened
sqlite bag. A successful Stop requires process exit and `metadata.yaml`.
Stop timeout retains ownership of the process and reports an error; shutdown
first attempts SIGINT, then bounded TERM/KILL cleanup if necessary. Forced
termination can leave an incomplete bag and is reported as an error.

The frontend reads controller state on mount/reconnect and subscribes to status.
It waits for confirmation, handles request timeouts/disconnections, ignores
superseded status responses, and disables commands when state is unknown.

## Verify

Local tests (no ROS required; process tests use a controlled test executable):

```bash
PYTHONPATH=airside/src/engine:$PYTHONPATH python3 -m unittest discover -s airside/src/engine/test -v
cd ims/frontend
pnpm test
pnpm lint
pnpm build
```

Real ROS smoke test, with the controller running, in a separate sourced terminal:

```bash
ros2 topic pub --rate 5 /rosbag_test std_msgs/msg/String '{data: "recording test"}'
```

In another sourced terminal:

```bash
ros2 service call /ims/rosbag/set_recording std_srvs/srv/SetBool '{data: true}'
ros2 service call /ims/rosbag/get_status std_srvs/srv/Trigger '{}'
# Allow test messages to arrive, then stop.
ros2 service call /ims/rosbag/set_recording std_srvs/srv/SetBool '{data: false}'
ros2 bag info /ros_ws/data/rosbags/<bag-directory-returned-by-the-controller>
```

Verify that `/rosbag_test` has recorded messages, and repeat Start/Stop to confirm
a new bag is created. Stop the test publisher with Ctrl+C. The real ROS smoke
test is also automated by `test_rosbag_integration.py`. It runs when ROS is
sourced and skips outside a ROS environment. It verifies service control,
idempotent Start, bag deserialization and restart in isolated ROS domain 194.
It was successfully run in
a local ROS 2 Humble container, alongside a Chrome Start/Stop/reload smoke test.
