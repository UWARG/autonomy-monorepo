// @vitest-environment jsdom
import {
  act,
  cleanup,
  fireEvent,
  render,
  screen,
  within,
  waitFor,
} from "@testing-library/react";
import { afterEach, beforeEach, expect, it, vi } from "vitest";
import { mockMessage, mockTopics } from "../src/mock";
const harness = vi.hoisted(() => ({
  urls: [] as string[],
  start: vi.fn(),
  stop: vi.fn(),
  subscribe: vi.fn(),
  discover: vi.fn(),
}));
vi.mock("../src/ros", () => ({
  LiveTransport: class {
    constructor(url: string) {
      harness.urls.push(url);
    }
    start = harness.start;
    subscribe = harness.subscribe;
    discover = harness.discover;
  },
}));
beforeEach(() => {
  vi.resetModules();
  harness.urls.length = 0;
  harness.start.mockReset().mockImplementation((update) => {
    update("connected");
    return harness.stop;
  });
  harness.stop.mockReset();
  harness.discover.mockReset().mockResolvedValue(mockTopics);
  harness.subscribe.mockReset().mockImplementation((name, _type, callback) => {
    if (name !== "/camera/image_raw") callback(mockMessage(name));
    return () => {};
  });
});
afterEach(() => {
  cleanup();
  window.history.replaceState({}, "", "/");
});

it("keeps the default endpoint and connection lifecycle, collapses settings, removes section navigation, and reuses panel samples in the overview", async () => {
  window.history.replaceState({}, "", "/");
  const { default: App } = await import("../src/App");
  const view = render(<App />);
  await screen.findByText("48.5 °C");
  expect(harness.urls).toEqual([
    `ws://${location.hostname || "localhost"}:9090`,
  ]);
  expect(harness.start).toHaveBeenCalledOnce();
  expect(harness.subscribe.mock.calls.map((c) => c[0]).sort()).toEqual([
    "/dashboard/temperatures",
    "/dashboard/tree",
    "/mavros/state",
    "/preflight/diagnostics",
  ]);
  const overview = screen.getByRole("region", { name: "System overview" });
  expect(within(overview).getByText("Connected")).toBeTruthy();
  expect(within(overview).getByText("Disarmed")).toBeTruthy();
  expect(within(overview).getByText("Warning")).toBeTruthy();
  expect(within(overview).getByText("MOCK / STABILIZE")).toBeTruthy();
  expect(
    screen.queryByRole("navigation", { name: "Monitoring sections" }),
  ).toBeNull();
  expect(view.container.querySelectorAll('a[href^="#"]')).toHaveLength(0);
  const settings = view.container.querySelector(".connection-settings")!;
  expect(settings.hasAttribute("open")).toBe(false);
  fireEvent.click(
    within(settings as HTMLElement).getByText(/Connection Settings/),
  );
  expect(settings.hasAttribute("open")).toBe(true);
  expect(screen.getByLabelText("rosbridge URL")).toBeTruthy();
  expect(
    screen.getByRole("link", { name: "Open mock demo" }).getAttribute("href"),
  ).toBe("?mode=mock");
  view.unmount();
  expect(harness.stop).toHaveBeenCalledOnce();
});

it("preserves custom simulation ports, validates the URL, and only changes transport after Connect", async () => {
  window.history.replaceState(
    {},
    "",
    "/?bridge=ws%3A%2F%2Flocalhost%3A9091&preflight=%2Fcustom%2Fchecks&image=%2Fcamera%2Fimage_raw",
  );
  const { default: App } = await import("../src/App");
  const view = render(<App />);
  await screen.findByText("48.5 °C");
  expect(harness.urls).toEqual(["ws://localhost:9091"]);
  expect(
    (screen.getByLabelText("rosbridge URL") as HTMLInputElement).value,
  ).toBe("ws://localhost:9091");
  expect(
    (screen.getByLabelText("Image topic") as HTMLSelectElement).value,
  ).toBe("/camera/image_raw");
  expect(screen.getByText(/Expected topic:/).textContent).toContain(
    "/custom/checks",
  );
  fireEvent.change(screen.getByLabelText("rosbridge URL"), {
    target: { value: "http://invalid" },
  });
  fireEvent.click(screen.getByRole("button", { name: "Connect" }));
  expect(screen.getByRole("alert").textContent).toBe(
    "Use a ws:// or wss:// URL",
  );
  expect(harness.urls).toHaveLength(1);
  fireEvent.change(screen.getByLabelText("rosbridge URL"), {
    target: { value: "ws://localhost:9191" },
  });
  expect(harness.urls).toHaveLength(1);
  fireEvent.click(screen.getByRole("button", { name: "Connect" }));
  expect(harness.urls).toEqual(["ws://localhost:9091", "ws://localhost:9191"]);
  expect(harness.stop).toHaveBeenCalledOnce();
  expect(view.container.querySelector(".endpoint code")?.textContent).toBe(
    "ws://localhost:9191",
  );
  await waitFor(() => expect(screen.queryByRole("alert")).toBeNull());
});

it("distinguishes reconnecting from disconnected, shows errors and masks the overview after disconnect", async () => {
  let update: (state: string, error?: string) => void = () => {};
  harness.start.mockImplementation((cb) => {
    update = cb;
    cb("connecting");
    return harness.stop;
  });
  const { default: App } = await import("../src/App");
  render(<App />);
  expect(screen.getByText("Connecting / retrying")).toBeTruthy();
  act(() => update("connected"));
  await screen.findByText("48.5 °C");
  act(() =>
    update(
      "disconnected",
      "WebSocket error; check URL, network and rosbridge.",
    ),
  );
  expect(screen.getByText(/WebSocket error;/)).toBeTruthy();
  expect(screen.getByText(/Topic list stale:/)).toBeTruthy();
  expect(screen.getByRole("heading", { name: "ROS 2 topics · 5" })).toBeTruthy();
  const overview = screen.getByRole("region", { name: "System overview" });
  expect(within(overview).queryByText("Disarmed")).toBeNull();
  expect(
    within(overview).getAllByText("Unavailable", { selector: "dd" }),
  ).toHaveLength(2);
  expect(within(overview).getAllByText("Unknown")).toHaveLength(3);
  act(() => update("connecting"));
  expect(screen.getByText("Connecting / retrying")).toBeTruthy();
  expect(harness.start).toHaveBeenCalledOnce();
});
