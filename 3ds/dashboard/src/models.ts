export interface TopicInfo {
  name: string;
  type: string;
}
export interface TreeNode {
  id: string;
  parent: string | null;
  name: string;
  status: string;
  feedback: string;
}
export interface TreeSnapshot {
  version: 1;
  nodes: TreeNode[];
  tip: string | null;
  stamp: number;
}
export interface Temperatures {
  version: 1;
  host: string;
  source: string;
  stamp: number;
  cpu: Reading[];
  gpu: Reading[];
}
export interface Reading {
  sensor: string;
  celsius: number;
}
export interface Check {
  name: string;
  level: number;
  message: string;
  values: { key: string; value: string }[];
}
export interface Diagnostics {
  checks: Check[];
  stamp: string;
}
export interface MavrosState {
  connected: boolean;
  armed: boolean;
  guided: boolean;
  mode: string;
  system_status: number;
}
export function object(value: unknown): Record<string, unknown> {
  if (!value || typeof value !== "object" || Array.isArray(value))
    throw Error("Expected an object");
  return value as Record<string, unknown>;
}
export function text(value: unknown): string {
  if (typeof value !== "string") throw Error("Expected text");
  return value;
}
export function number(value: unknown): number {
  if (typeof value !== "number" || !Number.isFinite(value))
    throw Error("Expected finite number");
  return value;
}
export function array(value: unknown): unknown[] {
  if (!Array.isArray(value)) throw Error("Expected array");
  return value;
}
export function jsonMessage(value: unknown): Record<string, unknown> {
  return object(JSON.parse(text(object(value).data)));
}
export function parseTree(value: unknown): TreeSnapshot {
  const v = jsonMessage(value);
  if (v.version !== 1) throw Error("Unsupported tree schema");
  const nodes = array(v.nodes).map((item) => {
    const n = object(item);
    const status = text(n.status);
    if (!["INVALID", "RUNNING", "SUCCESS", "FAILURE"].includes(status))
      throw Error("Invalid tree status");
    return {
      id: text(n.id),
      parent: n.parent === null ? null : text(n.parent),
      name: text(n.name),
      status,
      feedback: n.feedback === undefined ? "" : text(n.feedback),
    };
  });
  if (
    nodes.length > 500 ||
    new Set(nodes.map((n) => n.id)).size !== nodes.length
  )
    throw Error("Invalid tree size or IDs");
  const seen = new Set<string>();
  for (const node of nodes) {
    if (node.parent !== null && !seen.has(node.parent))
      throw Error("Missing or cyclic tree parent");
    seen.add(node.id);
  }
  return {
    version: 1,
    nodes,
    tip: v.tip === null ? null : text(v.tip),
    stamp: number(v.stamp),
  };
}
export function parseTemperatures(value: unknown): Temperatures {
  const v = jsonMessage(value);
  if (v.version !== 1) throw Error("Unsupported temperature schema");
  const readings = (items: unknown): Reading[] =>
    array(items).map((item) => {
      const r = object(item);
      const celsius = number(r.celsius);
      if (celsius < -50 || celsius > 200) throw Error("Invalid temperature");
      return { sensor: text(r.sensor), celsius };
    });
  return {
    version: 1,
    host: text(v.host),
    source: text(v.source),
    stamp: number(v.stamp),
    cpu: readings(v.cpu),
    gpu: readings(v.gpu),
  };
}
export function parseState(value: unknown): MavrosState {
  const v = object(value);
  if (
    typeof v.connected !== "boolean" ||
    typeof v.armed !== "boolean" ||
    typeof v.guided !== "boolean"
  )
    throw Error("Invalid MAVROS state");
  return {
    connected: v.connected,
    armed: v.armed,
    guided: v.guided,
    mode: text(v.mode),
    system_status: number(v.system_status),
  };
}
export function parseDiagnostics(value: unknown): Diagnostics {
  const v = object(value);
  const stamp = object(object(v.header).stamp);
  return {
    stamp: `${number(stamp.sec)}.${String(number(stamp.nanosec)).padStart(9, "0")} (ROS time)`,
    checks: array(v.status).map((item) => {
      const c = object(item);
      const level = number(c.level);
      if (![0, 1, 2, 3].includes(level))
        throw Error("Unknown diagnostic level");
      return {
        name: text(c.name),
        level,
        message: text(c.message),
        values: array(c.values).map((item) => {
          const kv = object(item);
          return { key: text(kv.key), value: text(kv.value) };
        }),
      };
    }),
  };
}
export function overall(checks: Check[]): string {
  if (!checks.length || checks.some((c) => c.level === 3)) return "Unavailable";
  return checks.some((c) => c.level === 2)
    ? "Failed"
    : checks.some((c) => c.level === 1)
      ? "Warning"
      : "Passed";
}

export interface GpsFix {
  latitude: number;
  longitude: number;
  altitude?: number;
}
export function parseGps(value: unknown): GpsFix {
  const message = object(value);
  if (number(object(message.status).status) < 0) throw Error("GPS has no fix");
  const latitude = number(message.latitude),
    longitude = number(message.longitude);
  if (Math.abs(latitude) > 90 || Math.abs(longitude) > 180)
    throw Error("Invalid GPS coordinates");
  return {
    latitude,
    longitude,
    altitude: message.altitude === null ? undefined : number(message.altitude),
  };
}
export function parseAttitude(value: unknown): {
  roll: number;
  pitch: number;
  yaw: number;
} {
  const message = object(value);
  if (array(message.orientation_covariance)[0] === -1)
    throw Error("IMU orientation unavailable");
  const q = object(message.orientation);
  const values = [number(q.x), number(q.y), number(q.z), number(q.w)];
  const norm = Math.hypot(...values);
  if (!Number.isFinite(norm) || norm < 1e-6)
    throw Error("Invalid orientation quaternion");
  const [x, y, z, w] = values.map((v) => v / norm);
  const degrees = 180 / Math.PI;
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * degrees,
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))) * degrees,
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * degrees,
  };
}
export function parsePosition(value: unknown): {
  x: number;
  y: number;
  z: number;
  frame: string;
} {
  const message = object(value),
    position = object(object(message.pose).position);
  return {
    x: number(position.x),
    y: number(position.y),
    z: number(position.z),
    frame: text(object(message.header).frame_id),
  };
}
