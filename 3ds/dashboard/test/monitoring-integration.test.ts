import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { fileURLToPath } from "node:url";
import { expect, it, vi } from "vitest";
import { LiveTransport } from "../src/ros";
import {
  parseTree,
  parseDiagnostics,
  parseTemperatures,
  overall,
} from "../src/models";

const execute = promisify(execFile);
it.skipIf(process.env.RUN_MONITORING_INTEGRATION !== "1")(
  "actual engine snapshots, diagnostic levels and real temperature node with labelled sensor fixtures",
  async () => {
    const docker = async (...args: string[]) =>
      (await execute("docker", args, { maxBuffer: 2_000_000 })).stdout.trim();
    let container = "",
      stop = () => {};
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
        "ROS_DOMAIN_ID=214",
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
      await docker("exec", container, "mkdir", "-p", "/tmp/monitor-src");
      await docker(
        "cp",
        fileURLToPath(new URL("../../src/engine/engine", import.meta.url)),
        `${container}:/tmp/monitor-src/engine`,
      );
      await docker(
        "cp",
        fileURLToPath(
          new URL(
            "../../src/dashboard_monitor/dashboard_monitor",
            import.meta.url,
          ),
        ),
        `${container}:/tmp/monitor-src/dashboard_monitor`,
      );
      await docker(
        "cp",
        fileURLToPath(new URL("./monitoring_fixture.py", import.meta.url)),
        `${container}:/tmp/monitoring_fixture.py`,
      );
      await docker(
        "exec",
        "-d",
        container,
        "sh",
        "-c",
        "/entrypoint.sh python3 -u /tmp/monitoring_fixture.py >/tmp/fixture.log 2>&1",
      );
      await vi.waitFor(
        async () =>
          expect(
            await docker(
              "exec",
              container,
              "test",
              "-f",
              "/tmp/monitor-validation/control.json",
            ),
          ).toBe(""),
        { timeout: 8000 },
      );
      await docker(
        "exec",
        "-d",
        "-e",
        "PYTHONPATH=/tmp/monitor-src",
        container,
        "sh",
        "-c",
        "/entrypoint.sh python3 -u -c 'import sys; sys.path.insert(0, \"/tmp/monitor-src\"); from engine.manager import main; main()' >/tmp/engine.log 2>&1",
      );
      await docker(
        "exec",
        "-d",
        "-e",
        "PYTHONPATH=/tmp/monitor-src",
        container,
        "sh",
        "-c",
        "/entrypoint.sh python3 -u -c 'import sys; sys.path.insert(0, \"/tmp/monitor-src\"); from dashboard_monitor.temperature_node import main; main()' --ros-args -p sys_root:=/tmp/monitor-validation/FAKE_SENSOR_FILES >/tmp/temperatures.log 2>&1",
      );
      const transport = new LiveTransport(`ws://${bridge}`);
      let connected = false;
      stop = transport.start((state) => {
        connected = state === "connected";
      });
      await vi.waitFor(() => expect(connected).toBe(true), { timeout: 15000 });
      let tree: ReturnType<typeof parseTree> | undefined,
        diagnostics: ReturnType<typeof parseDiagnostics> | undefined,
        temperatures: ReturnType<typeof parseTemperatures> | undefined;
      let treeCount = 0,
        diagnosticCount = 0;
      transport.subscribe("/dashboard/tree", "std_msgs/msg/String", (value) => {
        tree = parseTree(value);
        treeCount++;
      });
      transport.subscribe(
        "/preflight/diagnostics",
        "diagnostic_msgs/msg/DiagnosticArray",
        (value) => {
          diagnostics = parseDiagnostics(value);
          diagnosticCount++;
        },
      );
      transport.subscribe(
        "/dashboard/temperatures",
        "std_msgs/msg/String",
        (value) => {
          temperatures = parseTemperatures(value);
        },
      );
      await vi.waitFor(
        () => {
          expect(treeCount).toBeGreaterThanOrEqual(2);
          expect(
            tree!.nodes.find((n) => n.name === "ConfigureStreamRates")?.status,
          ).toBe("RUNNING");
          expect(tree!.nodes.length).toBeGreaterThan(10);
          expect(overall(diagnostics!.checks)).toBe("Warning");
          expect(temperatures!.cpu[0].celsius).toBe(48.5);
          expect(temperatures!.gpu[0].celsius).toBe(51);
          expect(temperatures!.source).toContain("FAKE_SENSOR_FILES");
        },
        { timeout: 20000 },
      );
      const control = async (level: number, enabled = true) =>
        docker(
          "exec",
          container,
          "python3",
          "-c",
          'import sys;from pathlib import Path;Path("/tmp/monitor-validation/control.json").write_text(sys.argv[1])',
          JSON.stringify({ level, enabled }),
        );
      for (const [level, status] of [
        [0, "Passed"],
        [2, "Failed"],
        [3, "Unavailable"],
      ] as const) {
        await control(level);
        await vi.waitFor(
          () => expect(overall(diagnostics!.checks)).toBe(status),
          { timeout: 6000 },
        );
      }
      const changeSensor = async (index: number, value: string) =>
        docker(
          "exec",
          container,
          "python3",
          "-c",
          "import sys;from pathlib import Path;Path(sys.argv[1]).write_text(sys.argv[2])",
          `/tmp/monitor-validation/FAKE_SENSOR_FILES/class/thermal/thermal_zone${index}/temp`,
          value,
        );
      await changeSensor(0, "60000");
      await vi.waitFor(() => expect(temperatures!.cpu[0].celsius).toBe(60), {
        timeout: 6000,
      });
      await changeSensor(0, "nan");
      await changeSensor(1, "invalid");
      await vi.waitFor(
        () => {
          expect(temperatures!.cpu).toEqual([]);
          expect(temperatures!.gpu).toEqual([]);
        },
        { timeout: 6000 },
      );
      await control(3, false);
      await new Promise((resolve) => setTimeout(resolve, 1200));
      const count = diagnosticCount;
      await new Promise((resolve) => setTimeout(resolve, 5500));
      expect(diagnosticCount).toBe(count);
      const nodes = await docker(
        "exec",
        container,
        "/entrypoint.sh",
        "ros2",
        "node",
        "list",
      );
      expect(nodes).toContain("/engine_manager");
      expect(nodes).not.toMatch(
        /\/mavros\/mavros|camera_node|position_controller/,
      );
      console.info(
        "PASS: real engine snapshots; diagnostic warning/pass/fail/unknown and stopped stream; real temperature node fixture changes/missing sensors. No FCU/hardware attached.",
      );
    } catch (error) {
      if (container) {
        console.error(
          await docker(
            "exec",
            container,
            "tail",
            "-n",
            "60",
            "/tmp/bridge.log",
          ),
        );
        console.error(
          await docker(
            "exec",
            container,
            "tail",
            "-n",
            "25",
            "/tmp/fixture.log",
            "/tmp/engine.log",
            "/tmp/temperatures.log",
          ),
        );
      }
      throw error;
    } finally {
      stop();
      if (container) await docker("rm", "-f", container);
    }
  },
  70000,
);
