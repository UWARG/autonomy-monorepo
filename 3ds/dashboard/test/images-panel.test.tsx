// @vitest-environment jsdom
import {
  act,
  cleanup,
  fireEvent,
  render,
  screen,
} from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { ImagesPanel } from "../src/components/Images";
import { mockTransport } from "../src/mock";
import type { MonitorProps } from "../src/components/Telemetry";

const draw = vi.fn();
beforeEach(() => {
  vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue({
    putImageData: draw,
  } as unknown as CanvasRenderingContext2D);
  vi.stubGlobal(
    "ImageData",
    class {
      constructor(
        public data: Uint8ClampedArray,
        public width: number,
        public height: number,
      ) {}
    },
  );
});
afterEach(() => {
  cleanup();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
  draw.mockClear();
});
function setup() {
  const callbacks = new Map<string, (value: unknown) => void>();
  const stops = [vi.fn(), vi.fn(), vi.fn()];
  const subscribe = vi.fn(
    (name: string, _type: string, callback: (value: unknown) => void) => {
      callbacks.set(name, callback);
      return stops[subscribe.mock.calls.length - 1];
    },
  );
  const props: MonitorProps = {
    transport: { ...mockTransport, subscribe },
    state: "connected",
    topics: [
      { name: "/a", type: "sensor_msgs/msg/Image" },
      { name: "/b", type: "sensor_msgs/msg/Image" },
    ],
    now: Date.now(),
    preflight: "/preflight/diagnostics",
  };
  const view = render(<ImagesPanel {...props} />);
  return { callbacks, stops, subscribe, props, view };
}
describe("image rendering and subscription lifecycle", () => {
  it("renders a valid black frame as receiving, with correct canvas dimensions", () => {
    const { callbacks } = setup();
    fireEvent.change(screen.getByRole("combobox"), { target: { value: "/a" } });
    act(() =>
      callbacks.get("/a")!({
        width: 2,
        height: 1,
        step: 6,
        encoding: "rgb8",
        data: "AAAAAAAA",
      }),
    );
    expect(screen.queryByText(/No signal/)).toBeNull();
    expect(screen.getByText(/Receiving frames/)).toBeTruthy();
    const canvas = screen.getByLabelText(
      "Latest ROS camera frame",
    ) as HTMLCanvasElement;
    expect(canvas.width).toBe(2);
    expect(canvas.height).toBe(1);
    expect(draw).toHaveBeenCalledOnce();
    expect([...draw.mock.calls[0][0].data]).toEqual([
      0, 0, 0, 255, 0, 0, 0, 255,
    ]);
  });
  it("does not churn subscriptions, switches topics, disconnects, reconnects and unmounts", () => {
    const { subscribe, stops, props, view } = setup();
    fireEvent.change(screen.getByRole("combobox"), { target: { value: "/a" } });
    view.rerender(<ImagesPanel {...props} now={props.now + 1000} />);
    expect(subscribe).toHaveBeenCalledTimes(1);
    fireEvent.change(screen.getByRole("combobox"), { target: { value: "/b" } });
    expect(stops[0]).toHaveBeenCalledOnce();
    expect(subscribe.mock.calls[1][0]).toBe("/b");
    view.rerender(<ImagesPanel {...props} state="disconnected" />);
    expect(stops[1]).toHaveBeenCalledOnce();
    expect(screen.getByText(/Disconnected/)).toBeTruthy();
    view.rerender(<ImagesPanel {...props} state="connected" />);
    expect(subscribe).toHaveBeenCalledTimes(3);
    view.unmount();
    expect(stops[2]).toHaveBeenCalledOnce();
  });
  it("reports unsupported encodings and expires the receiving indicator", () => {
    const { callbacks, props, view } = setup();
    fireEvent.change(screen.getByRole("combobox"), { target: { value: "/a" } });
    act(() =>
      callbacks.get("/a")!({
        width: 1,
        height: 1,
        step: 2,
        encoding: "16UC1",
        data: [0, 0],
      }),
    );
    expect(screen.getByText(/Unsupported encoding/)).toBeTruthy();
    act(() =>
      callbacks.get("/a")!({
        width: 1,
        height: 1,
        step: 3,
        encoding: "rgb8",
        data: [0, 0, 0],
      }),
    );
    view.rerender(<ImagesPanel {...props} now={Date.now() + 6000} />);
    expect(screen.getByText(/STALE/)).toBeTruthy();
    expect(screen.queryByText(/Receiving frames/)).toBeNull();
  });
});

it("labels historical frames and never reports a failed compressed image as receiving", () => {
  const { props, view, callbacks } = setup();
  view.rerender(
    <ImagesPanel
      {...props}
      topics={[{ name: "/jpeg", type: "sensor_msgs/msg/CompressedImage" }]}
    />,
  );
  fireEvent.change(screen.getByRole("combobox"), {
    target: { value: "/jpeg" },
  });
  act(() => callbacks.get("/jpeg")!({ format: "jpeg", data: [255, 216, 255] }));
  expect(screen.getByText(/Receiving frames/)).toBeTruthy();
  fireEvent.error(screen.getByAltText("Latest ROS camera frame"));
  expect(screen.getByRole("alert").textContent).toBe(
    "Unable to decode compressed image",
  );
  expect(screen.queryByText(/Receiving frames/)).toBeNull();
  expect(screen.getByText("Last received frame · not current")).toBeTruthy();
});
