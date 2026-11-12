import { describe, it, expect } from "vitest";
import { parseGps, parseAttitude, parsePosition } from "../src/models";
describe("live flight telemetry parsing", () => {
  it("preserves real GPS coordinates and ellipsoid altitude", () => {
    expect(
      parseGps({
        status: { status: 0 },
        latitude: -35.3632621,
        longitude: 149.1652374,
        altitude: 603.64,
      }),
    ).toEqual({
      latitude: -35.3632621,
      longitude: 149.1652374,
      altitude: 603.64,
    });
  });
  it("rejects missing fixes and invalid coordinates", () => {
    expect(() => parseGps({ status: { status: -1 } })).toThrow("no fix");
    expect(() =>
      parseGps({
        status: { status: 0 },
        latitude: 91,
        longitude: 1,
        altitude: 0,
      }),
    ).toThrow();
    expect(() =>
      parseGps({
        status: { status: 0 },
        latitude: null,
        longitude: 1,
        altitude: 0,
      }),
    ).toThrow();
  });
  it("converts and normalizes a ROS quaternion without inventing orientation", () => {
    expect(
      parseAttitude({
        orientation_covariance: [1],
        orientation: { x: 0, y: 0, z: -Math.SQRT1_2, w: -Math.SQRT1_2 },
      }).yaw,
    ).toBeCloseTo(90);
    expect(() => parseAttitude({ orientation_covariance: [-1] })).toThrow(
      "unavailable",
    );
    expect(() =>
      parseAttitude({
        orientation_covariance: [1],
        orientation: { x: 0, y: 0, z: 0, w: 0 },
      }),
    ).toThrow();
  });
  it("retains local coordinates and their declared frame", () => {
    expect(
      parsePosition({
        header: { frame_id: "map" },
        pose: { position: { x: 1, y: 2, z: 0.19 } },
      }),
    ).toEqual({ x: 1, y: 2, z: 0.19, frame: "map" });
  });
});
