import { useState, type ReactNode } from "react";
import { EmptyState, StatusLine, OverviewMetric } from "./Presentation";
import { useSubscription, sampleStatus } from "../hooks";
import {
  parseTree,
  parseState,
  parseTemperatures,
  parseDiagnostics,
  overall,
  parseGps,
  parseAttitude,
  parsePosition,
} from "../models";
import type { TopicInfo } from "../models";
import type { Transport, Connection } from "../ros";
export interface MonitorProps {
  transport: Transport;
  state: Connection;
  topics: TopicInfo[];
  now: number;
  preflight: string;
  overview?: HTMLElement | null;
}
export function Panel({
  title,
  children,
  id,
}: {
  title: string;
  children: ReactNode;
  id: string;
}) {
  return (
    <section id={id} className="panel">
      <div className="panel-heading">
        <h2>{title}</h2>
        <span className="panel-index" aria-hidden="true">
          {
            (
              {
                images: "01",
                mavros: "02",
                tree: "03",
                preflight: "04",
                temperatures: "05",
                topics: "06",
              } as Record<string, string>
            )[id]
          }
        </span>
      </div>
      {children}
    </section>
  );
}
function useMonitor<T>(
  props: MonitorProps,
  name: string,
  type: string,
  parse: (value: unknown) => T,
) {
  const missing = !props.topics.some((t) => t.name === name && t.type === type);
  const sample = useSubscription(
    props.transport,
    props.state,
    missing ? "" : name,
    type,
    parse,
  );
  const valid =
    props.state === "connected" &&
    !missing &&
    !sample.error &&
    !!sample.received &&
    props.now - sample.received <= 5000;
  return {
    value: sample.value,
    valid,
    status: sampleStatus(
      sample,
      props.now,
      props.state === "connected",
      missing,
    ),
  };
}
export function TreePanel(props: MonitorProps) {
  const data = useMonitor(
    props,
    "/dashboard/tree",
    "std_msgs/msg/String",
    parseTree,
  );
  const [collapsed, setCollapsed] = useState<Set<string>>(new Set());
  const depths = new Map<string, number>();
  const hidden = new Set<string>();
  const parents = new Set(data.value?.nodes.map((n) => n.parent));
  for (const node of data.value?.nodes ?? []) {
    depths.set(node.id, node.parent ? (depths.get(node.parent) ?? 0) + 1 : 0);
    if (node.parent && (collapsed.has(node.parent) || hidden.has(node.parent)))
      hidden.add(node.id);
  }
  return (
    <Panel title="Behavior tree" id="tree">
      <StatusLine status={data.status} />
      {data.value ? (
        <>
          <p className="mission-tip">
            Mission tip:{" "}
            <strong>
              {data.valid
                ? (data.value.nodes.find((n) => n.id === data.value?.tip)
                    ?.name ?? "No active tip")
                : "Unknown (last snapshot below)"}
            </strong>
          </p>
          <ul
            className={`tree ${data.valid ? "current" : "historical"}`}
            aria-label="Mission hierarchy"
          >
            {data.value.nodes
              .filter((n) => !hidden.has(n.id))
              .map((n) => (
                <li
                  key={n.id}
                  style={{
                    paddingLeft: Math.min(depths.get(n.id) ?? 0, 12) * 12,
                  }}
                >
                  <div className="tree-node">
                    {parents.has(n.id) ? (
                      <button
                        className="branch-toggle"
                        aria-label={`${collapsed.has(n.id) ? "Expand" : "Collapse"} ${n.name}`}
                        aria-expanded={!collapsed.has(n.id)}
                        onClick={() =>
                          setCollapsed((old) => {
                            const next = new Set(old);
                            if (next.has(n.id)) next.delete(n.id);
                            else next.add(n.id);
                            return next;
                          })
                        }
                      >
                        {collapsed.has(n.id) ? "+" : "−"}
                      </button>
                    ) : (
                      <span className="branch-leaf" aria-hidden="true">
                        └
                      </span>
                    )}
                    <span className="node-name">
                      {n.name}
                      {n.feedback && <small>{n.feedback}</small>}
                    </span>
                  </div>
                  <b className={`status ${n.status.toLowerCase()}`}>
                    {n.status}
                  </b>
                </li>
              ))}
          </ul>
          <p className="meta source-note">
            Publisher time: {data.value.stamp.toFixed(3)} s (ROS)
          </p>
        </>
      ) : (
        <EmptyState title="No engine snapshot">
          Engine activity will appear when a snapshot is received.
        </EmptyState>
      )}
    </Panel>
  );
}
export function MavrosPanel(props: MonitorProps) {
  const data = useMonitor(
    props,
    "/mavros/state",
    "mavros_msgs/msg/State",
    parseState,
  );
  const v = data.valid ? data.value : undefined;
  return (
    <Panel title="Flight controller · MAVROS" id="mavros">
      <OverviewMetric
        target={props.overview}
        label="FCU link"
        value={v ? (v.connected ? "Connected" : "Disconnected") : "Unknown"}
      />
      <OverviewMetric
        target={props.overview}
        label="Arming"
        value={v?.connected ? (v.armed ? "Armed" : "Disarmed") : "Unknown"}
      />
      <OverviewMetric
        target={props.overview}
        label="Flight mode"
        value={v?.connected ? v.mode : "Unknown"}
      />
      <StatusLine status={data.status} />
      <div className="instrument-group">
        <h3>Flight-controller state</h3>
        <dl className="readings flight-state">
          <ReactRow
            label="FCU link"
            value={v ? (v.connected ? "Connected" : "Disconnected") : "Unknown"}
          />
          <ReactRow
            label="Arming"
            value={v?.connected ? (v.armed ? "Armed" : "Disarmed") : "Unknown"}
          />
          <ReactRow
            label="Flight mode"
            value={v?.connected ? v.mode : "Unknown"}
          />
          <ReactRow
            label="Guided flag"
            value={v?.connected ? String(v.guided) : "Unknown"}
          />
          <ReactRow
            label="MAV system status"
            value={v?.connected ? String(v.system_status) : "Unknown"}
          />
        </dl>
      </div>
      <FlightTelemetry {...props} />
      <p className="meta source-note">
        FCU state is independent of the browser’s rosbridge connection. No
        heartbeat latency is inferred.
      </p>
    </Panel>
  );
}
function FlightTelemetry(props: MonitorProps) {
  const gps = useMonitor(
    props,
    "/mavros/global_position/global",
    "sensor_msgs/msg/NavSatFix",
    parseGps,
  );
  const imu = useMonitor(
    props,
    "/mavros/imu/data",
    "sensor_msgs/msg/Imu",
    parseAttitude,
  );
  const pose = useMonitor(
    props,
    "/mavros/local_position/pose",
    "geometry_msgs/msg/PoseStamped",
    parsePosition,
  );
  return (
    <>
      <OverviewMetric
        target={props.overview}
        label="GPS"
        value={gps.valid ? "Fix received" : "Unavailable"}
      />
      <div className="instrument-group">
        <div className="instrument-heading">
          <h3>GPS</h3>
          <StatusLine status={gps.status} />
        </div>
        <dl className="readings navigation-readings">
          <ReactRow
            label="Latitude"
            value={gps.valid ? `${gps.value!.latitude.toFixed(7)}°` : "Unknown"}
          />
          <ReactRow
            label="Longitude"
            value={
              gps.valid ? `${gps.value!.longitude.toFixed(7)}°` : "Unknown"
            }
          />
          <ReactRow
            label="Altitude · WGS84 ellipsoid"
            value={
              gps.valid && gps.value?.altitude !== undefined
                ? `${gps.value.altitude.toFixed(2)} m`
                : "Unknown"
            }
          />
        </dl>
      </div>
      <div className="instrument-group">
        <div className="instrument-heading">
          <h3>Attitude · ROS frame</h3>
          <StatusLine status={imu.status} />
        </div>
        <dl className="readings three-readings">
          {(["roll", "pitch", "yaw"] as const).map((axis) => (
            <ReactRow
              key={axis}
              label={axis}
              value={imu.valid ? `${imu.value![axis].toFixed(1)}°` : "Unknown"}
            />
          ))}
        </dl>
      </div>
      <div className="instrument-group">
        <div className="instrument-heading">
          <h3>Local position{pose.valid ? ` · ${pose.value!.frame}` : ""}</h3>
          <StatusLine status={pose.status} />
        </div>
        <dl className="readings three-readings">
          {(["x", "y", "z"] as const).map((axis) => (
            <ReactRow
              key={axis}
              label={axis.toUpperCase()}
              value={
                pose.valid ? `${pose.value![axis].toFixed(2)} m` : "Unknown"
              }
            />
          ))}
        </dl>
      </div>
    </>
  );
}
function ReactRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="reading">
      <dt>{label}</dt>
      <dd>{value}</dd>
    </div>
  );
}
export function PreflightPanel(props: MonitorProps) {
  const data = useMonitor(
    props,
    props.preflight,
    "diagnostic_msgs/msg/DiagnosticArray",
    parseDiagnostics,
  );
  const result =
    data.valid && data.value ? overall(data.value.checks) : "Unavailable";
  return (
    <Panel title="Preflight diagnostics" id="preflight">
      <OverviewMetric
        target={props.overview}
        label="Preflight"
        value={result}
      />
      <StatusLine status={data.status} />
      <div className={`diagnostic-result result-${result.toLowerCase()}`}>
        <span aria-hidden="true">
          {result === "Passed"
            ? "✓"
            : result === "Failed"
              ? "×"
              : result === "Warning"
                ? "!"
                : "—"}
        </span>
        <h3>{result}</h3>
      </div>
      {!data.value?.checks.length && (
        <EmptyState title="Preflight checks unavailable">
          No diagnostic results have been received.
        </EmptyState>
      )}
      {data.value?.checks.map((check, i) => (
        <div className="check" key={`${check.name}-${i}`}>
          <div className="check-heading">
            <strong>{check.name}</strong>
            <span
              className={`status ${data.valid ? `level-${check.level}` : "historical"}`}
            >
              {data.valid
                ? ["Passed", "Warning", "Failed", "Unavailable"][check.level]
                : "Unknown (historical)"}
            </span>
          </div>
          <p>{check.message}</p>
          {check.values.map((kv, j) => (
            <p className="meta" key={j}>
              {kv.key}: {kv.value}
            </p>
          ))}
        </div>
      ))}
      {data.value && (
        <p className="meta source-note">Source stamp: {data.value.stamp}</p>
      )}
      <details className="technical-details">
        <summary>Technical details</summary>
        <p className="meta">
          Expected topic: <code>{props.preflight}</code> ·
          diagnostic_msgs/msg/DiagnosticArray
        </p>
        <p className="meta">
          Issue #210 supplies the actual checks. Missing, stale or invalid
          results never establish readiness.
        </p>
      </details>
    </Panel>
  );
}
export function TemperaturePanel(props: MonitorProps) {
  const data = useMonitor(
    props,
    "/dashboard/temperatures",
    "std_msgs/msg/String",
    parseTemperatures,
  );
  return (
    <Panel title="Companion temperatures" id="temperatures">
      <StatusLine status={data.status} />
      <div className="temperature-grid">
        {(["cpu", "gpu"] as const).map((kind) => (
          <div className="temperature-instrument" key={kind}>
            <h3>{kind.toUpperCase()}</h3>
            {data.valid && data.value?.[kind].length ? (
              data.value[kind].map((r) => (
                <div className="sensor-reading" key={r.sensor}>
                  <strong>{r.celsius.toFixed(1)} °C</strong>
                  <span className="meta">{r.sensor}</span>
                </div>
              ))
            ) : (
              <p className="sensor-unavailable">
                Unavailable · no current measured sensor
              </p>
            )}
          </div>
        ))}
      </div>
      <p className="meta source-note">
        Source:{" "}
        {data.value
          ? `${data.value.host} · ${data.value.source}`
          : "Awaiting monitoring node; browser temperatures are never used."}
      </p>
    </Panel>
  );
}
