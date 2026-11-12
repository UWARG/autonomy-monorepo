import { describe, it, expect } from "vitest";
import {
  parseTree,
  parseTemperatures,
  parseState,
  parseDiagnostics,
  overall,
} from "../src/models";
import { mockMessage } from "../src/mock";
import { parseImage } from "../src/images";
import { sampleStatus } from "../src/hooks";
describe("real wire-format adapters", () => {
  it("parses mock fixtures using the live parsers", () => {
    expect(parseTree(mockMessage("/dashboard/tree")).nodes).toHaveLength(2);
    expect(
      parseTemperatures(mockMessage("/dashboard/temperatures")).cpu[0].celsius,
    ).toBe(48.5);
    expect(parseState(mockMessage("/mavros/state")).armed).toBe(false);
    expect(
      overall(parseDiagnostics(mockMessage("/preflight/diagnostics")).checks),
    ).toBe("Warning");
  });
  it("never treats absent or stale checks as passed", () => {
    expect(overall([])).toBe("Unavailable");
    expect(overall([{ name: "x", level: 3, message: "", values: [] }])).toBe(
      "Unavailable",
    );
  });
  it("rejects malformed and incompatible messages", () => {
    for (const parse of [
      parseTree,
      parseTemperatures,
      parseState,
      parseDiagnostics,
    ])
      for (const value of [
        null,
        {},
        { data: "invalid" },
        { data: '{"version":2}' },
      ])
        expect(() => parse(value)).toThrow();
    expect(() =>
      parseState({
        connected: "true",
        armed: false,
        guided: false,
        mode: "X",
        system_status: 3,
      }),
    ).toThrow();
  });
  it("distinguishes missing, stale, invalid and disconnected samples", () => {
    const sample = { times: [], received: 1000, value: 1 };
    expect(sampleStatus(sample, 8000, true)).toContain("STALE");
    expect(sampleStatus(sample, 8000, false)).toBe("Disconnected");
    expect(sampleStatus(sample, 8000, true, true)).toBe("Topic unavailable");
    expect(sampleStatus({ times: [] }, 8000, true)).toBe("Waiting for data");
    expect(sampleStatus({ ...sample, error: "bad" }, 8000, true)).toContain(
      "Invalid",
    );
  });
});
describe("bounded image conversion", () => {
  it("converts padded BGR rows into RGBA", () => {
    const image = parseImage({
      width: 1,
      height: 2,
      step: 4,
      encoding: "bgr8",
      data: [1, 2, 3, 99, 4, 5, 6, 99],
    });
    expect([...image.pixels!]).toEqual([3, 2, 1, 255, 6, 5, 4, 255]);
  });
  it("supports mono8, RGBA and base64", () => {
    expect([
      ...parseImage({
        width: 1,
        height: 1,
        step: 1,
        encoding: "mono8",
        data: btoa(String.fromCharCode(7)),
      }).pixels!,
    ]).toEqual([7, 7, 7, 255]);
    expect([
      ...parseImage({
        width: 1,
        height: 1,
        step: 4,
        encoding: "rgba8",
        data: [1, 2, 3, 4],
      }).pixels!,
    ]).toEqual([1, 2, 3, 4]);
    expect(
      parseImage({
        format: "rgb8; jpeg compressed bgr8",
        data: [255, 216, 255],
      }).src,
    ).toContain("data:image/jpeg;base64,");
  });
  it("rejects unsupported, oversized and truncated input", () => {
    expect(() =>
      parseImage({
        width: 1,
        height: 1,
        step: 1,
        encoding: "constructor",
        data: [1],
      }),
    ).toThrow();
    for (const value of [
      { width: 1, height: 1, step: 2, encoding: "16UC1", data: [0, 0] },
      { width: 999999, height: 99, step: 999999, encoding: "mono8", data: [] },
      { width: 1, height: 1, step: 3, encoding: "rgb8", data: [1] },
      { format: "16UC1; compressedDepth png", data: [] },
    ])
      expect(() => parseImage(value)).toThrow();
  });
});
