import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { fileURLToPath } from "node:url";
import { expect, it, vi } from "vitest";
import { LiveTransport } from "../src/ros";
import { parseImage } from "../src/images";
import { parseState } from "../src/models";

const execute = promisify(execFile);
// Explicit opt-in only. Ordinary unit tests never need Docker or a ROS graph.
it.skipIf(process.env.RUN_ROS_DASHBOARD_INTEGRATION !== "1")(
  "receives real Humble rosbridge fragments from isolated ROS fixtures",
  async () => {
    let container = "",
      stop = () => {};
    const docker = async (...args: string[]) =>
      (await execute("docker", args, { maxBuffer: 2_000_000 })).stdout.trim();
    try {
      container = await docker(
        "run",
        "--rm",
        "-d",
        "--network",
        "bridge",
        "-e",
        "ROS_LOCALHOST_ONLY=1",
        "-e",
        "ROS_DOMAIN_ID=213",
        "-p",
        "127.0.0.1::9090",
        "warg/3ds-dashboard-verify:issue213",
        "sleep",
        "infinity",
      );
      const bridge = await docker("port", container, "9090/tcp");
      // Exercise the current bridge-only launch, even with an older local image.
      await docker(
        "cp",
        fileURLToPath(
          new URL(
            "../../src/airside_bringup/launch/dashboard.launch.py",
            import.meta.url,
          ),
        ),
        `${container}:/tmp/dashboard.launch.py`,
      );
      await docker(
        "exec",
        "-d",
        container,
        "sh",
        "-c",
        "/entrypoint.sh ros2 launch /tmp/dashboard.launch.py bridge_bind:=0.0.0.0 temperature_enabled:=false >/tmp/bridge.log 2>&1",
      );
      await docker(
        "cp",
        fileURLToPath(new URL("./ros_fixture.py", import.meta.url)),
        `${container}:/tmp/dashboard_fixture.py`,
      );
      await docker(
        "exec",
        "-d",
        container,
        "/entrypoint.sh",
        "python3",
        "-u",
        "/tmp/dashboard_fixture.py",
      );
      const transport = new LiveTransport(`ws://${bridge}`);
      let connected = false;
      stop = transport.start((state) => {
        connected = state === "connected";
      });
      await vi.waitFor(() => expect(connected).toBe(true), { timeout: 10000 });
      await vi.waitFor(
        async () => {
          const topics = await transport.discover();
          expect(topics).toContainEqual({
            name: "/dashboard_test/image",
            type: "sensor_msgs/msg/Image",
          });
        },
        { timeout: 10000, interval: 500 },
      );
      let state: ReturnType<typeof parseState> | undefined;
      transport.subscribe(
        "/dashboard_test/state",
        "mavros_msgs/msg/State",
        (value) => {
          state = parseState(value);
        },
      );
      await vi.waitFor(
        () =>
          expect(state).toMatchObject({
            connected: true,
            armed: false,
            mode: "TEST_FIXTURE",
          }),
        { timeout: 5000 },
      );

      let liveFrame: ReturnType<typeof parseImage> | undefined;
      transport.subscribe(
        "/dashboard_test/image",
        "sensor_msgs/msg/Image",
        (value) => {
          liveFrame = parseImage(value);
        },
      );
      await vi.waitFor(
        () => expect(liveFrame).toMatchObject({ width: 640, height: 480 }),
        { timeout: 10000 },
      );
      expect([...liveFrame!.pixels!.slice(0, 4)]).toEqual([0, 0, 0, 255]);

      // This isolated graph has exactly one large publication: our image.
      // Confirm the real server fragmented it and LiveTransport delivered it,
      // without introducing a second high-bandwidth client into this assertion.
      const logs = await docker("exec", container, "cat", "/tmp/bridge.log");
      expect(logs).toContain("Subscribed to /dashboard_test/image");
      expect(logs).toMatch(/sending 2 parts \[fragment size: 1000000/);
      const nodes = await docker(
        "exec",
        container,
        "/entrypoint.sh",
        "ros2",
        "node",
        "list",
      );
      expect(nodes).not.toMatch(
        /engine_manager|mavros|camera_node|position_controller/,
      );
      console.info(
        "Verified real rosapi discovery, MAVROS-type fixture, and",
        "real fragmented image delivery; no mission/hardware nodes.",
      );
    } catch (error) {
      if (container) {
        console.error(
          await docker(
            "exec",
            container,
            "tail",
            "-n",
            "50",
            "/tmp/bridge.log",
          ),
        );
        console.error(
          await docker(
            "exec",
            container,
            "/entrypoint.sh",
            "ros2",
            "topic",
            "info",
            "/dashboard_test/image",
            "-v",
          ),
        );
      }
      throw error;
    } finally {
      stop();
      if (container) await docker("rm", "-f", container);
    }
  },
  45000,
);
