import { describe, expect, it, vi } from "vitest";
import { RosbridgeDecoder, ROSBRIDGE_LIMITS } from "../src/rosbridge";
import { parseImage } from "../src/images";

function fragments(message: unknown, id = "frame", total = 2) {
  const serialized = JSON.stringify(message);
  const width = Math.ceil(serialized.length / total);
  return Array.from({ length: total }, (_, num) =>
    JSON.stringify({
      op: "fragment",
      id,
      num,
      total,
      data: serialized.slice(num * width, (num + 1) * width),
    }),
  );
}
const image = {
  op: "publish",
  topic: "/camera/image_raw",
  msg: { width: 2, height: 1, step: 6, encoding: "rgb8", data: "AQIDBAUG" },
};
const state = {
  op: "publish",
  topic: "/mavros/state",
  msg: {
    connected: true,
    armed: false,
    guided: false,
    mode: "STABILIZE",
    system_status: 3,
  },
};

describe("bounded rosbridge JSON fragment reassembly", () => {
  it("preserves ordinary publications and service responses", () => {
    const decoder = new RosbridgeDecoder(vi.fn());
    expect(decoder.decode(JSON.stringify(state))).toEqual(state);
    const response = {
      op: "service_response",
      id: "topics",
      result: true,
      values: { topics: [], types: [] },
    };
    expect(decoder.decode(JSON.stringify(response))).toEqual(response);
  });
  it("reassembles two parts, ignores identical duplicates and decodes RGB8", () => {
    const report = vi.fn(),
      decoder = new RosbridgeDecoder(report);
    const [first, second] = fragments(image);
    expect(decoder.decode(first)).toBeUndefined();
    expect(decoder.decode(first)).toBeUndefined();
    const result = decoder.decode(second)!;
    expect(result).toEqual(image);
    expect([...parseImage(result.msg).pixels!]).toEqual([
      1, 2, 3, 255, 4, 5, 6, 255,
    ]);
    expect(report).not.toHaveBeenCalled();
    expect(decoder.pendingGroups).toBe(0);
    expect(decoder.bufferedChars).toBe(0);
  });
  it("handles out-of-order and interleaved messages without mixing topics", () => {
    const decoder = new RosbridgeDecoder(vi.fn());
    const camera = fragments(image, "camera", 3),
      mavros = fragments(state, "mavros");
    expect(decoder.decode(camera[2])).toBeUndefined();
    expect(decoder.decode(mavros[1])).toBeUndefined();
    expect(decoder.decode(camera[0])).toBeUndefined();
    expect(decoder.decode(mavros[0])).toEqual(state);
    expect(decoder.decode(camera[1])).toEqual(image);
  });
  it("expires incomplete groups even when no more packets arrive", () => {
    let now = 100;
    const report = vi.fn(),
      decoder = new RosbridgeDecoder(report, () => now);
    decoder.decode(fragments(image)[0]);
    expect(decoder.pendingGroups).toBe(1);
    now += ROSBRIDGE_LIMITS.retentionMs;
    decoder.expire();
    expect(decoder.pendingGroups).toBe(0);
    expect(decoder.bufferedChars).toBe(0);
    expect(report).toHaveBeenCalledWith(expect.stringContaining("expired"));
  });
  it("does not extend retention with duplicate fragments", () => {
    let now = 0;
    const decoder = new RosbridgeDecoder(vi.fn(), () => now);
    const first = fragments(image)[0];
    decoder.decode(first);
    now = 9999;
    decoder.decode(first);
    now = 10000;
    decoder.expire();
    expect(decoder.pendingGroups).toBe(0);
  });
  it.each([
    { num: -1, total: 2, data: "x" },
    { num: 2, total: 2, data: "x" },
    { num: 0.5, total: 2, data: "x" },
    { num: 0, total: 0, data: "x" },
    { num: 0, total: 257, data: "x" },
    { num: 0, total: 1.5, data: "x" },
    { num: 0, total: 2, data: [] },
    { num: "0", total: 2, data: "x" },
  ])("rejects invalid fragment metadata: %j", (metadata) => {
    const report = vi.fn(),
      decoder = new RosbridgeDecoder(report);
    expect(
      decoder.decode(
        JSON.stringify({ op: "fragment", id: "bad", ...metadata }),
      ),
    ).toBeUndefined();
    expect(report).toHaveBeenCalledOnce();
    expect(decoder.bufferedChars).toBe(0);
  });
  it("rejects malformed JSON, bad envelopes, binary packets and IDs", () => {
    const report = vi.fn(),
      decoder = new RosbridgeDecoder(report);
    for (const raw of [
      "invalid",
      "null",
      "[]",
      "{}",
      new ArrayBuffer(2),
      JSON.stringify({ op: "fragment", id: 4, num: 0, total: 1, data: "{}" }),
    ]) {
      expect(() => decoder.decode(raw)).not.toThrow();
      expect(decoder.decode(raw)).toBeUndefined();
    }
    expect(report).toHaveBeenCalled();
    expect(decoder.decode(JSON.stringify(state))).toEqual(state);
  });
  it("rejects malformed dispatch fields before they reach roslib", () => {
    const report = vi.fn(),
      decoder = new RosbridgeDecoder(report);
    for (const message of [
      { op: "publish", topic: {}, msg: {} },
      { op: "publish", topic: "/x", msg: null },
      { op: "service_response", id: 5 },
      { op: "call_service", service: [] },
    ])
      expect(decoder.decode(JSON.stringify(message))).toBeUndefined();
    expect(report).toHaveBeenCalledTimes(4);
  });
  it("drops conflicting totals, conflicting duplicates, malformed and nested assemblies", () => {
    const report = vi.fn(),
      decoder = new RosbridgeDecoder(report);
    const [first] = fragments(image);
    for (const changed of [
      { num: 0, total: 3, data: "x" },
      { num: 0, total: 2, data: "different" },
    ]) {
      decoder.decode(first);
      decoder.decode(
        JSON.stringify({ op: "fragment", id: "frame", ...changed }),
      );
      expect(decoder.pendingGroups).toBe(0);
    }
    decoder.decode(
      JSON.stringify({
        op: "fragment",
        id: "bad-json",
        num: 0,
        total: 1,
        data: "not json",
      }),
    );
    decoder.decode(
      fragments(
        { op: "fragment", id: "nested", num: 0, total: 1, data: "{}" },
        "outer",
        1,
      )[0],
    );
    expect(decoder.pendingGroups).toBe(0);
    expect(decoder.bufferedChars).toBe(0);
    expect(report).toHaveBeenCalledTimes(4);
  });
  it("bounds group count, per-message size and aggregate retained data", () => {
    const report = vi.fn();
    const decoder = new RosbridgeDecoder(report, () => 0, {
      ...ROSBRIDGE_LIMITS,
      maxGroups: 2,
      maxMessageChars: 500,
      maxBufferedChars: 8,
    });
    const part = (id: string, data: string) =>
      JSON.stringify({ op: "fragment", id, num: 0, total: 2, data });
    decoder.decode(part("a", "abcd"));
    decoder.decode(part("b", "abcd"));
    decoder.decode(part("c", "x"));
    expect(decoder.pendingGroups).toBe(2);
    decoder.decode(
      JSON.stringify({ op: "fragment", id: "b", num: 1, total: 2, data: "x" }),
    );
    expect(decoder.pendingGroups).toBe(1);
    expect(decoder.bufferedChars).toBe(4);
    decoder.reset();
    expect(decoder.bufferedChars).toBe(0);
    const small = new RosbridgeDecoder(report, () => 0, {
      ...ROSBRIDGE_LIMITS,
      maxMessageChars: 200,
    });
    small.decode(part("large", "x".repeat(120)));
    expect(small.pendingGroups).toBe(1);
    small.decode(
      JSON.stringify({
        op: "fragment",
        id: "large",
        num: 1,
        total: 2,
        data: "x".repeat(120),
      }),
    );
    expect(small.pendingGroups).toBe(0);
    small.decode("x".repeat(201));
    expect(report).toHaveBeenCalledWith(expect.stringContaining("size limit"));
  });
  it("does not join fragments across connection resets", () => {
    const decoder = new RosbridgeDecoder(vi.fn());
    const [first, second] = fragments(image);
    decoder.decode(first);
    decoder.reset();
    expect(decoder.decode(second)).toBeUndefined();
    decoder.reset();
    expect(decoder.decode(first)).toBeUndefined();
    expect(decoder.decode(second)).toEqual(image);
  });
  it("accepts a real-sized 640x480 black base64 image fragmented at 1,000,000 chars", () => {
    const decoder = new RosbridgeDecoder(vi.fn());
    const publication = {
      op: "publish",
      topic: "/camera/image_raw",
      msg: {
        width: 640,
        height: 480,
        step: 1920,
        encoding: "rgb8",
        data: Buffer.alloc(640 * 480 * 3).toString("base64"),
      },
    };
    const serialized = JSON.stringify(publication);
    expect(serialized.length).toBeGreaterThan(1_000_000);
    let complete: Record<string, unknown> | undefined;
    const total = Math.ceil(serialized.length / 1_000_000);
    for (let num = 0; num < total; num++)
      complete = decoder.decode(
        JSON.stringify({
          op: "fragment",
          id: "full-frame",
          num,
          total,
          data: serialized.slice(num * 1_000_000, (num + 1) * 1_000_000),
        }),
      );
    const decoded = parseImage(complete!.msg);
    expect(decoded).toMatchObject({ width: 640, height: 480 });
    expect(decoded.pixels).toHaveLength(640 * 480 * 4);
    expect([...decoded.pixels!.slice(0, 4)]).toEqual([0, 0, 0, 255]);
    expect([...decoded.pixels!.slice(-4)]).toEqual([0, 0, 0, 255]);
    expect(decoder.pendingGroups).toBe(0);
  });
});
