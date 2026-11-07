// @vitest-environment jsdom
import {
  act,
  cleanup,
  fireEvent,
  render,
  renderHook,
  screen,
} from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import { useSubscription } from "../src/hooks";
import type { Transport } from "../src/ros";
import { mockMessage, mockTopics, mockTransport } from "../src/mock";
import { MavrosPanel, PreflightPanel } from "../src/components/Telemetry";
import { TopicsPanel } from "../src/components/Topics";
const identity = (value: unknown) => value;
afterEach(() => {
  cleanup();
  vi.useRealTimers();
});
describe("React monitoring lifecycle", () => {
  it("cleans up on topic changes, disconnects and unmount", () => {
    const stops = [vi.fn(), vi.fn()];
    let calls = 0;
    const transport: Transport = {
      ...mockTransport,
      subscribe: vi.fn((_name, _type, callback) => {
        callback({ data: "x" });
        return stops[calls++];
      }),
    };
    const hook = renderHook(
      ({ name, state }) =>
        useSubscription(
          transport,
          state,
          name,
          "std_msgs/msg/String",
          identity,
        ),
      {
        initialProps: {
          name: "/a",
          state: "connected" as "connected" | "disconnected",
        },
      },
    );
    expect(hook.result.current.value).toEqual({ data: "x" });
    hook.rerender({ name: "/b", state: "connected" });
    expect(stops[0]).toHaveBeenCalledOnce();
    hook.rerender({ name: "/b", state: "disconnected" });
    expect(stops[1]).toHaveBeenCalledOnce();
    expect(hook.result.current.value).toBeUndefined();
    hook.unmount();
  });
  it("does not accept malformed telemetry or updates after cleanup", () => {
    let callback: ((value: unknown) => void) | undefined;
    const transport: Transport = {
      ...mockTransport,
      subscribe: (_n, _t, cb) => {
        callback = cb;
        return vi.fn();
      },
    };
    const reject = () => {
      throw Error("invalid");
    };
    const hook = renderHook(() =>
      useSubscription(transport, "connected", "/a", "x", reject),
    );
    act(() => callback!({}));
    expect(hook.result.current.error).toContain("invalid");
    hook.unmount();
    act(() => callback!({}));
  });
  it("shows unknown FCU state after telemetry goes stale", () => {
    vi.useFakeTimers();
    vi.setSystemTime(10000);
    const props = {
      transport: {
        ...mockTransport,
        subscribe: (_n: string, _t: string, cb: (value: unknown) => void) => {
          cb(mockMessage("/mavros/state"));
          return () => {};
        },
      },
      state: "connected" as const,
      topics: mockTopics,
      now: 10000,
      preflight: "/preflight/diagnostics",
    };
    const view = render(<MavrosPanel {...props} />);
    expect(screen.getByText("Disarmed")).toBeTruthy();
    view.rerender(<MavrosPanel {...props} now={17000} />);
    expect(screen.queryByText("Disarmed")).toBeNull();
    expect(screen.getAllByText("Unknown").length).toBeGreaterThan(0);
  });
  it("never passes preflight without diagnostics", () => {
    render(
      <PreflightPanel
        transport={{ ...mockTransport, subscribe: () => () => {} }}
        state="connected"
        topics={[]}
        now={10000}
        preflight="/preflight/diagnostics"
      />,
    );
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(
      "Unavailable",
    );
    expect(screen.queryByText("Passed")).toBeNull();
  });
  it("subscribes to the selected inspector topic and cleans up", () => {
    const stop = vi.fn(),
      subscribe = vi.fn(() => stop);
    const view = render(
      <TopicsPanel
        transport={{ ...mockTransport, subscribe }}
        state="connected"
        topics={mockTopics}
        now={Date.now()}
        preflight="/preflight/diagnostics"
      />,
    );
    fireEvent.change(screen.getByRole("combobox"), {
      target: { value: "/mavros/state" },
    });
    expect(subscribe).toHaveBeenCalledWith(
      "/mavros/state",
      "mavros_msgs/msg/State",
      expect.any(Function),
      500,
    );
    view.unmount();
    expect(stop).toHaveBeenCalledOnce();
  });
});

describe("complete browser DOM mock view", () => {
  it("renders every panel with the explicit synthetic banner and cleans timers", async () => {
    window.history.replaceState({}, "", "/?mode=mock");
    const { default: App } = await import("../src/App");
    const view = render(<App />);
    expect(await screen.findByText(/All telemetry is synthetic/)).toBeTruthy();
    for (const name of [
      "Behavior tree",
      "Flight controller · MAVROS",
      "Preflight diagnostics",
      "Companion temperatures",
      "Image preview",
      "ROS 2 topics · 5",
    ])
      expect(screen.getByRole("heading", { name })).toBeTruthy();
    expect(await screen.findByText("48.5 °C")).toBeTruthy();
    expect(screen.queryByText("Connect")).toBeNull();
    view.unmount();
    window.history.replaceState({}, "", "/");
  });
});

it("renders actual GPS/attitude/position shapes and masks stale readings", () => {
  vi.useFakeTimers();
  vi.setSystemTime(10000);
  const samples: Record<string, unknown> = {
    "/mavros/state": mockMessage("/mavros/state"),
    "/mavros/global_position/global": {
      status: { status: 0 },
      latitude: -35.3632621,
      longitude: 149.1652374,
      altitude: 603.64,
    },
    "/mavros/imu/data": {
      orientation_covariance: [1],
      orientation: { x: 0, y: 0, z: Math.SQRT1_2, w: Math.SQRT1_2 },
    },
    "/mavros/local_position/pose": {
      header: { frame_id: "map" },
      pose: { position: { x: 0, y: 0, z: 0.19 } },
    },
  };
  const topics = [
    ...mockTopics,
    {
      name: "/mavros/global_position/global",
      type: "sensor_msgs/msg/NavSatFix",
    },
    { name: "/mavros/imu/data", type: "sensor_msgs/msg/Imu" },
    {
      name: "/mavros/local_position/pose",
      type: "geometry_msgs/msg/PoseStamped",
    },
  ];
  const transport: Transport = {
    ...mockTransport,
    subscribe: (name, _type, callback) => {
      callback(samples[name]);
      return () => {};
    },
  };
  const props = {
    transport,
    topics,
    state: "connected" as const,
    now: 10000,
    preflight: "/preflight/diagnostics",
  };
  const view = render(<MavrosPanel {...props} />);
  expect(screen.getByText("-35.3632621°")).toBeTruthy();
  expect(screen.getByText("90.0°")).toBeTruthy();
  expect(screen.getByText("0.19 m")).toBeTruthy();
  view.rerender(<MavrosPanel {...props} now={18000} />);
  expect(screen.queryByText("-35.3632621°")).toBeNull();
  expect(screen.queryByText("90.0°")).toBeNull();
});
