// @vitest-environment jsdom
import { act, cleanup, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  PreflightPanel,
  TemperaturePanel,
  TreePanel,
  type MonitorProps,
} from "../src/components/Telemetry";
import { mockTransport } from "../src/mock";
const callbacks = new Map<string, (value: unknown) => void>();
function props(): MonitorProps {
  return {
    transport: {
      ...mockTransport,
      subscribe: (name, _type, callback) => {
        callbacks.set(name, callback);
        return () => callbacks.delete(name);
      },
    },
    state: "connected",
    now: 10000,
    preflight: "/preflight/diagnostics",
    topics: [
      { name: "/dashboard/tree", type: "std_msgs/msg/String" },
      { name: "/dashboard/temperatures", type: "std_msgs/msg/String" },
      {
        name: "/preflight/diagnostics",
        type: "diagnostic_msgs/msg/DiagnosticArray",
      },
    ],
  };
}
function emit(name: string, message: unknown) {
  act(() => callbacks.get(name)!(message));
}
function diagnostic(level: number) {
  return {
    header: { stamp: { sec: 123, nanosec: 0 } },
    status: [
      {
        name: "TEST FIXTURE: camera",
        level,
        message: "Synthetic test only",
        values: [{ key: "fixture", value: "true" }],
      },
    ],
  };
}
beforeEach(() => {
  callbacks.clear();
  vi.useFakeTimers();
  vi.setSystemTime(10000);
});
afterEach(() => {
  cleanup();
  vi.useRealTimers();
});

describe("preflight rendering and honest unavailable states", () => {
  it.each([
    [0, "Passed"],
    [1, "Warning"],
    [2, "Failed"],
    [3, "Unavailable"],
  ] as const)("renders diagnostic level %i as %s", (level, label) => {
    render(<PreflightPanel {...props()} />);
    emit("/preflight/diagnostics", diagnostic(level));
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(label);
    expect(screen.getByText("Synthetic test only")).toBeTruthy();
    expect(screen.getByText("fixture: true")).toBeTruthy();
  });
  it("turns a prior pass into unavailable when stale, invalid, empty or disconnected", () => {
    const p = props(),
      view = render(<PreflightPanel {...p} />);
    emit("/preflight/diagnostics", diagnostic(0));
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(
      "Passed",
    );
    view.rerender(<PreflightPanel {...p} now={16000} />);
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(
      "Unavailable",
    );
    expect(screen.queryByText("Passed")).toBeNull();
    view.rerender(<PreflightPanel {...p} />);
    emit("/preflight/diagnostics", {});
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(
      "Unavailable",
    );
    emit("/preflight/diagnostics", {
      header: { stamp: { sec: 123, nanosec: 0 } },
      status: [],
    });
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(
      "Unavailable",
    );
    view.rerender(<PreflightPanel {...p} state="disconnected" />);
    expect(screen.getByRole("heading", { level: 3 }).textContent).toBe(
      "Unavailable",
    );
  });
});

describe("behavior tree status visualization", () => {
  it.each(["RUNNING", "SUCCESS", "FAILURE", "INVALID"])(
    "renders actual node status %s and parent order",
    (status) => {
      render(<TreePanel {...props()} />);
      emit("/dashboard/tree", {
        data: JSON.stringify({
          version: 1,
          stamp: 123,
          tip: "child",
          nodes: [
            {
              id: "root",
              parent: null,
              name: "Root",
              status: "RUNNING",
              feedback: "",
            },
            {
              id: "child",
              parent: "root",
              name: "TEST behavior",
              status,
              feedback: "test feedback",
            },
          ],
        }),
      });
      expect(screen.getAllByText(status).length).toBeGreaterThan(0);
      expect(screen.getByText(/test feedback/)).toBeTruthy();
      expect(screen.getByText(/Publisher time: 123/)).toBeTruthy();
    },
  );
  it("marks old snapshots historical when they stop arriving", () => {
    const p = props(),
      view = render(<TreePanel {...p} />);
    emit("/dashboard/tree", {
      data: JSON.stringify({
        version: 1,
        stamp: 123,
        tip: "root",
        nodes: [{ id: "root", parent: null, name: "Root", status: "RUNNING" }],
      }),
    });
    view.rerender(<TreePanel {...p} now={16000} />);
    expect(screen.getByText(/STALE/)).toBeTruthy();
    expect(screen.getByText(/Unknown \(last snapshot below\)/)).toBeTruthy();
  });
});

describe("temperature provenance and sensor availability", () => {
  const message = (cpu: unknown[], gpu: unknown[]) => ({
    data: JSON.stringify({
      version: 1,
      stamp: 123,
      host: "TEST FIXTURE HOST",
      source: "FAKE_SENSOR_FILES",
      cpu,
      gpu,
    }),
  });
  it("renders separate Celsius sensors and source; clears stale and missing readings", () => {
    const p = props(),
      view = render(<TemperaturePanel {...p} />);
    emit(
      "/dashboard/temperatures",
      message(
        [{ sensor: "TEST CPU", celsius: 48.5 }],
        [{ sensor: "TEST GPU", celsius: 51 }],
      ),
    );
    expect(screen.getByText("48.5 °C")).toBeTruthy();
    expect(screen.getByText("51.0 °C")).toBeTruthy();
    expect(
      screen.getByText(/TEST FIXTURE HOST.*FAKE_SENSOR_FILES/),
    ).toBeTruthy();
    view.rerender(<TemperaturePanel {...p} now={16000} />);
    expect(screen.queryByText("48.5 °C")).toBeNull();
    view.rerender(<TemperaturePanel {...p} />);
    emit("/dashboard/temperatures", message([], []));
    expect(screen.queryByText("51.0 °C")).toBeNull();
    expect(
      screen.getAllByText(/Unavailable.*no current measured sensor/),
    ).toHaveLength(2);
  });
  it("does not show a previously valid number after a malformed update", () => {
    render(<TemperaturePanel {...props()} />);
    emit(
      "/dashboard/temperatures",
      message([{ sensor: "TEST CPU", celsius: 48.5 }], []),
    );
    emit("/dashboard/temperatures", { data: "invalid" });
    expect(screen.queryByText("48.5 °C")).toBeNull();
    expect(screen.getByText(/Invalid data/)).toBeTruthy();
  });
});
