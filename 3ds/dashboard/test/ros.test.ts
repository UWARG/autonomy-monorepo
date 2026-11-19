import { beforeEach, afterEach, describe, it, expect, vi } from "vitest";
const fake = vi.hoisted(() => ({
  handlers: new Map<string, (...args: unknown[]) => void>(),
  connect: vi.fn(),
  close: vi.fn(),
  subscribe: vi.fn(),
  unsubscribe: vi.fn(),
  getTopics: vi.fn(),
  send: vi.fn(),
  decoder: undefined as
    undefined | ((raw: unknown, dispatch: (message: unknown) => void) => void),
}));
vi.mock("roslib", () => ({
  default: {
    Ros: class {
      constructor(options: {
        transportOptions: { decoder: typeof fake.decoder };
      }) {
        fake.decoder = options.transportOptions.decoder;
      }
      emit(name: string, value: unknown) {
        fake.handlers.get(name)?.(value);
      }
      isConnected = true;
      callOnConnection = fake.send;
      on(name: string, cb: (...args: unknown[]) => void) {
        fake.handlers.set(name, cb);
      }
      off(name: string) {
        fake.handlers.delete(name);
      }
      connect = fake.connect;
      close = fake.close;
      getTopics = fake.getTopics;
    },
    Topic: class {
      subscribe = fake.subscribe;
      unsubscribe = fake.unsubscribe;
    },
  },
}));
import { LiveTransport, backoff } from "../src/ros";
describe("connection lifecycle", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.clearAllMocks();
    fake.handlers.clear();
  });
  afterEach(() => {
    vi.useRealTimers();
  });
  it("reconnects with bounded backoff and cancels on cleanup", () => {
    const transport = new LiveTransport("ws://localhost:9090"),
      update = vi.fn(),
      stop = transport.start(update);
    expect(fake.connect).toHaveBeenCalledTimes(1);
    fake.handlers.get("close")!();
    vi.advanceTimersByTime(1000);
    expect(fake.connect).toHaveBeenCalledTimes(2);
    fake.handlers.get("close")!();
    vi.advanceTimersByTime(1999);
    expect(fake.connect).toHaveBeenCalledTimes(2);
    vi.advanceTimersByTime(1);
    expect(fake.connect).toHaveBeenCalledTimes(3);
    fake.handlers.get("connection")!();
    expect(update).toHaveBeenLastCalledWith("connected");
    fake.handlers.get("close")!();
    stop();
    vi.runAllTimers();
    expect(fake.connect).toHaveBeenCalledTimes(3);
    expect(fake.close).toHaveBeenCalledOnce();
    expect(fake.handlers.size).toBe(0);
    expect(backoff(50)).toBe(30000);
  });
  it("unsubscribes the exact callback", () => {
    const transport = new LiveTransport("ws://localhost:9090"),
      callback = vi.fn();
    const stop = transport.subscribe(
      "/camera/image_raw",
      "sensor_msgs/msg/Image",
      callback,
    );
    fake.handlers.get("/camera/image_raw")!({ width: 1 });
    expect(callback).toHaveBeenCalledWith({ width: 1 });
    expect(fake.send).toHaveBeenLastCalledWith(
      expect.objectContaining({
        op: "subscribe",
        throttle_rate: 500,
        queue_length: 1,
      }),
    );
    stop();
    expect(fake.handlers.has("/camera/image_raw")).toBe(false);
    expect(fake.send).toHaveBeenLastCalledWith(
      expect.objectContaining({ op: "unsubscribe" }),
    );
  });
  it("shares subscriptions and keeps monitoring when an inspector unmounts", () => {
    const transport = new LiveTransport("ws://localhost:9090"),
      first = vi.fn(),
      second = vi.fn();
    const a = transport.subscribe("/x", "x", first),
      b = transport.subscribe("/x", "x", second);
    expect(fake.send).toHaveBeenCalledTimes(1);
    a();
    fake.handlers.get("/x")!({ data: 1 });
    expect(second).toHaveBeenCalledOnce();
    b();
    expect(fake.handlers.has("/x")).toBe(false);
  });
  it("handles browser CloseEvent without rendering an event as an error", () => {
    const update = vi.fn(),
      stop = new LiveTransport("ws://localhost:9090").start(update);
    fake.handlers.get("close")!({ code: 1006 });
    expect(update).toHaveBeenLastCalledWith("disconnected", undefined);
    stop();
  });
  it("clears partial assemblies on disconnection and cancels idle expiry on cleanup", () => {
    const update = vi.fn(),
      transport = new LiveTransport("ws://localhost:9090"),
      stop = transport.start(update);
    const dispatch = vi.fn();
    const serialized = JSON.stringify({
      op: "publish",
      topic: "/x",
      msg: { data: "new" },
    });
    const split = Math.floor(serialized.length / 2);
    const part = (num: number) =>
      JSON.stringify({
        op: "fragment",
        id: "reused",
        num,
        total: 2,
        data: num === 0 ? serialized.slice(0, split) : serialized.slice(split),
      });
    fake.decoder!(part(0), dispatch);
    fake.handlers.get("close")!();
    fake.handlers.get("connection")!();
    fake.decoder!(part(1), dispatch);
    expect(dispatch).not.toHaveBeenCalled();
    fake.handlers.get("close")!();
    fake.handlers.get("connection")!();
    fake.decoder!(part(0), dispatch);
    fake.decoder!(part(1), dispatch);
    expect(dispatch).toHaveBeenCalledOnce();
    fake.decoder!(part(0), dispatch);
    vi.advanceTimersByTime(10000);
    expect(update).toHaveBeenLastCalledWith(
      "connected",
      expect.stringContaining("expired"),
    );
    stop();
    expect(vi.getTimerCount()).toBe(0);
  });
  it("reports malformed protocol data without disconnecting healthy telemetry", () => {
    const update = vi.fn(),
      stop = new LiveTransport("ws://localhost:9090").start(update),
      dispatch = vi.fn();
    fake.decoder!("not json", dispatch);
    expect(update).toHaveBeenLastCalledWith(
      "connected",
      expect.stringContaining("receive error"),
    );
    fake.decoder!(
      JSON.stringify({ op: "publish", topic: "/x", msg: { data: "ok" } }),
      dispatch,
    );
    expect(dispatch).toHaveBeenCalledOnce();
    stop();
  });
  it("reports unavailable rosapi on timeout", async () => {
    const promise = new LiveTransport("ws://localhost:9090").discover();
    const assertion = expect(promise).rejects.toThrow("timed out");
    await vi.advanceTimersByTimeAsync(5000);
    await assertion;
    expect(fake.handlers.size).toBe(0);
  });
});
