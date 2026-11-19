// @vitest-environment jsdom
import {
  act,
  cleanup,
  fireEvent,
  render,
  screen,
  within,
} from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { TopicsPanel } from "../src/components/Topics";
import { TreePanel, type MonitorProps } from "../src/components/Telemetry";
import { mockMessage, mockTopics, mockTransport } from "../src/mock";

const topics = [
  ...mockTopics,
  { name: "/robot/camera/image", type: "sensor_msgs/msg/Image" },
  { name: "/clock", type: "rosgraph_msgs/msg/Clock" },
  { name: "/gazebo/camera", type: "sensor_msgs/msg/CompressedImage" },
];
function setup() {
  const callbacks = new Map<string, (value: unknown) => void>();
  const stops: ReturnType<typeof vi.fn>[] = [];
  const subscribe = vi.fn(
    (name: string, _type: string, callback: (value: unknown) => void) => {
      callbacks.set(name, callback);
      const stop = vi.fn();
      stops.push(stop);
      return stop;
    },
  );
  const discover = vi.fn(mockTransport.discover);
  const props: MonitorProps = {
    transport: { ...mockTransport, subscribe, discover },
    topics,
    state: "connected",
    now: 10000,
    preflight: "/preflight/diagnostics",
  };
  return { props, subscribe, discover, callbacks, stops };
}
beforeEach(() => {
  vi.useFakeTimers();
  vi.setSystemTime(10000);
});
afterEach(() => {
  cleanup();
  vi.useRealTimers();
  vi.unstubAllGlobals();
});

describe("topic browser presentation without ROS side effects", () => {
  it("searches case-insensitively by namespace, partial/full name and message type, with count and clear action", () => {
    const { props, subscribe, discover } = setup();
    render(<TopicsPanel {...props} />);
    const input = screen.getByRole("searchbox", {
      name: "Search ROS topics...",
    });
    for (const query of [
      "MAVROS",
      "/mavros/st",
      "/mavros/state",
      "MAVROS_MSGS/MSG/STATE",
    ]) {
      fireEvent.change(input, { target: { value: query } });
      expect(screen.getByText("1 of 8 topics")).toBeTruthy();
      expect(
        screen.getByRole("button", {
          name: "/mavros/state mavros_msgs/msg/State",
        }),
      ).toBeTruthy();
    }
    fireEvent.change(input, { target: { value: "not-a-topic" } });
    expect(screen.getByText("No matching topics")).toBeTruthy();
    expect(screen.getByText("0 of 8 topics")).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "Clear search" }));
    expect(screen.getByText("8 of 8 topics")).toBeTruthy();
    expect(subscribe).not.toHaveBeenCalled();
    expect(discover).not.toHaveBeenCalled();
  });
  it("groups by real namespaces, keeps ambiguous namespaces intact, and offers a sorted All Topics view", () => {
    const { props, subscribe } = setup();
    const { container } = render(<TopicsPanel {...props} />);
    const summaries = [...container.querySelectorAll(".topic-group summary")];
    expect(summaries.map((s) => s.textContent)).toEqual([
      "Root topics1",
      "Camera1",
      "Dashboard telemetry2",
      "Gazebo1",
      "MAVROS1",
      "Preflight1",
      "Namespace /robot1",
    ]);
    const robot = summaries.find((s) =>
      s.textContent?.includes("Namespace /robot"),
    )!.parentElement!;
    expect(
      within(robot).getByRole("button", {
        name: "/robot/camera/image sensor_msgs/msg/Image",
      }),
    ).toBeTruthy();
    // Native details toggling is presentation only; no subscriptions or requests.
    fireEvent.click(summaries[0]);
    expect(summaries[0].parentElement?.hasAttribute("open")).toBe(false);
    fireEvent.click(screen.getByRole("button", { name: "All Topics" }));
    expect(container.querySelectorAll(".topic-group")).toHaveLength(0);
    const names = [...container.querySelectorAll(".topic-option code")].map(
      (n) => n.textContent,
    );
    expect(names).toEqual(
      [...topics]
        .sort((a, b) => a.name.localeCompare(b.name))
        .map((t) => t.name),
    );
    expect(subscribe).not.toHaveBeenCalled();
  });
  it("keeps the selected subscription during search and grouping; releases it on selection change, removal and unmount", () => {
    const { props, subscribe, callbacks, stops } = setup();
    const view = render(<TopicsPanel {...props} />);
    fireEvent.click(
      screen.getByRole("button", {
        name: "/mavros/state mavros_msgs/msg/State",
      }),
    );
    act(() => callbacks.get("/mavros/state")!(mockMessage("/mavros/state")));
    expect(screen.getByLabelText("Latest topic message").textContent).toContain(
      '"armed": false',
    );
    expect(screen.getByText(/Browser-received rate/)).toBeTruthy();
    fireEvent.change(screen.getByRole("searchbox"), {
      target: { value: "camera" },
    });
    fireEvent.click(screen.getByRole("button", { name: "All Topics" }));
    expect(subscribe).toHaveBeenCalledTimes(1);
    expect(stops[0]).not.toHaveBeenCalled();
    expect(screen.getByLabelText("Latest topic message").textContent).toContain(
      '"armed": false',
    );
    fireEvent.change(screen.getByRole("combobox"), {
      target: { value: "/clock" },
    });
    expect(stops[0]).toHaveBeenCalledOnce();
    expect(subscribe).toHaveBeenCalledTimes(2);
    expect(
      screen.getByLabelText("Latest topic message").textContent,
    ).not.toContain('"armed": false');
    view.rerender(
      <TopicsPanel
        {...props}
        topics={props.topics.filter((t) => t.name !== "/clock")}
      />,
    );
    expect(stops[1]).toHaveBeenCalledOnce();
    expect(screen.getByText("Topic unavailable")).toBeTruthy();
    fireEvent.change(screen.getByRole("combobox"), {
      target: { value: "/mavros/state" },
    });
    view.unmount();
    expect(stops[2]).toHaveBeenCalledOnce();
  });
  it("labels old JSON stale and disconnected; reports clipboard failure without hiding the data", async () => {
    const { props, callbacks } = setup();
    const view = render(<TopicsPanel {...props} />);
    fireEvent.change(screen.getByRole("combobox"), {
      target: { value: "/clock" },
    });
    act(() => callbacks.get("/clock")!({ clock: { sec: 1, nanosec: 0 } }));
    view.rerender(<TopicsPanel {...props} now={17000} />);
    expect(screen.getByText(/STALE/)).toBeTruthy();
    expect(screen.getByLabelText("Latest topic message").textContent).toContain(
      '"sec": 1',
    );
    vi.stubGlobal("navigator", {
      clipboard: { writeText: vi.fn().mockRejectedValue(Error("denied")) },
    });
    await act(async () =>
      fireEvent.click(screen.getByRole("button", { name: "Copy JSON" })),
    );
    expect(screen.getByText(/Copy unavailable/)).toBeTruthy();
    view.rerender(<TopicsPanel {...props} state="disconnected" />);
    expect(screen.getByText("Disconnected")).toBeTruthy();
    expect(
      screen.getByLabelText("Latest topic message").textContent,
    ).not.toContain('"sec": 1');
    vi.restoreAllMocks();
  });
  it("has a useful disconnected empty state and does not subscribe before selection", () => {
    const { props, subscribe } = setup();
    render(<TopicsPanel {...props} topics={[]} state="disconnected" />);
    expect(screen.getByText("No topics available")).toBeTruthy();
    expect(
      screen.getByText("Connect to rosbridge to browse topics."),
    ).toBeTruthy();
    expect(screen.getByText("Select a topic to inspect")).toBeTruthy();
    expect(subscribe).not.toHaveBeenCalled();
  });
});

it("collapses tree branches without changing the engine subscription and marks historical snapshots", () => {
  const { props, callbacks, subscribe } = setup();
  const view = render(<TreePanel {...props} />);
  act(() => callbacks.get("/dashboard/tree")!(mockMessage("/dashboard/tree")));
  fireEvent.click(
    screen.getByRole("button", { name: "Collapse Root (fixture)" }),
  );
  expect(
    within(screen.getByRole("list", { name: "Mission hierarchy" })).queryByText(
      "WaitForSetpoint",
    ),
  ).toBeNull();
  fireEvent.click(
    screen.getByRole("button", { name: "Expand Root (fixture)" }),
  );
  expect(screen.getAllByText("WaitForSetpoint").length).toBeGreaterThan(0);
  expect(subscribe).toHaveBeenCalledTimes(1);
  view.rerender(<TreePanel {...props} now={17000} />);
  expect(screen.getByText(/STALE/)).toBeTruthy();
  expect(screen.getByText("Unknown (last snapshot below)")).toBeTruthy();
});
