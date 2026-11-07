import { useState } from "react";
import { useSubscription, sampleStatus } from "../hooks";
import type { TopicInfo } from "../models";
import { Panel, type MonitorProps } from "./Telemetry";
import { EmptyState, StatusLine } from "./Presentation";
const inspect = (value: unknown) =>
  JSON.stringify(value, null, 2)?.slice(0, 16000) ?? "null";

// Group by the actual first namespace, not guesses about a topic's purpose.
function namespace(topic: TopicInfo) {
  const parts = topic.name.split("/").filter(Boolean);
  return parts.length > 1 ? `/${parts[0]}` : "/";
}
const labels: Record<string, string> = {
  "/mavros": "MAVROS",
  "/camera": "Camera",
  "/dashboard": "Dashboard telemetry",
  "/preflight": "Preflight",
  "/gazebo": "Gazebo",
  "/simulation": "Simulation",
  "/": "Root topics",
};
export function TopicsPanel(props: MonitorProps) {
  const [selected, setSelected] = useState("");
  const [search, setSearch] = useState("");
  const [grouped, setGrouped] = useState(true);
  const [copyStatus, setCopyStatus] = useState("");
  const topic = props.topics.find((t) => t.name === selected);
  const sample = useSubscription(
    props.transport,
    props.state,
    topic?.name ?? "",
    topic?.type ?? "",
    inspect,
    500,
  );
  const times = sample.times.filter((t) => props.now - t < 10000);
  const hz =
    times.length > 1
      ? ((times.length - 1) * 1000) / (times[times.length - 1] - times[0])
      : undefined;
  const query = search.trim().toLowerCase();
  const sorted = [...props.topics].sort((a, b) => a.name.localeCompare(b.name));
  const filtered = sorted.filter((t) =>
    `${t.name} ${t.type}`.toLowerCase().includes(query),
  );
  const groups = new Map<string, TopicInfo[]>();
  for (const t of filtered) {
    const key = namespace(t);
    groups.set(key, [...(groups.get(key) ?? []), t]);
  }
  const copy = async (value: string, label: string) => {
    try {
      await navigator.clipboard.writeText(value);
      setCopyStatus(`${label} copied`);
    } catch {
      setCopyStatus("Copy unavailable. Select the text and copy manually.");
    }
  };
  const topicRows = (topics: TopicInfo[]) => (
    <ul className="topic-list">
      {topics.map((t) => (
        <li key={t.name}>
          <button
            className={`topic-option ${t.name === selected ? "selected" : ""}`}
            aria-pressed={t.name === selected}
            onClick={() => {
              setSelected(t.name);
              setCopyStatus("");
            }}
          >
            <code>{t.name}</code>
            <small>{t.type}</small>
          </button>
        </li>
      ))}
    </ul>
  );
  const status = sampleStatus(
    sample,
    props.now,
    props.state === "connected",
    !!selected && !topic,
  );
  return (
    <Panel title={`ROS 2 topics · ${props.topics.length}`} id="topics">
      <div className="topics-workspace">
        <div className="topic-browser">
          <label className="search-label" htmlFor="topic-search">
            Search ROS topics...
          </label>
          <div className="search-field">
            <input
              id="topic-search"
              type="search"
              placeholder="Name, namespace or message type"
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              autoComplete="off"
              spellCheck={false}
            />
            {search && (
              <button
                className="clear-search"
                onClick={() => setSearch("")}
                aria-label="Clear search"
              >
                ×
              </button>
            )}
          </div>
          <div className="topic-toolbar">
            <p className="meta" role="status">
              {filtered.length} of {props.topics.length} topics
            </p>
            <div className="view-switch" aria-label="Topic organization">
              <button aria-pressed={grouped} onClick={() => setGrouped(true)}>
                Namespaces
              </button>
              <button aria-pressed={!grouped} onClick={() => setGrouped(false)}>
                All Topics
              </button>
            </div>
          </div>
          <div className="topic-results">
            {!filtered.length ? (
              <EmptyState
                title={query ? "No matching topics" : "No topics available"}
              >
                {query
                  ? "Try another name or message type, or clear your search."
                  : props.state === "connected"
                    ? "Waiting for discovered ROS topics."
                    : "Connect to rosbridge to browse topics."}
              </EmptyState>
            ) : grouped ? (
              [...groups.entries()]
                .sort(([a], [b]) => a.localeCompare(b))
                .map(([key, topics]) => (
                  <details className="topic-group" key={key} open>
                    <summary>
                      <span>{labels[key] ?? `Namespace ${key}`}</span>
                      <span className="group-count">{topics.length}</span>
                    </summary>
                    {topicRows(topics)}
                  </details>
                ))
            ) : (
              topicRows(filtered)
            )}
          </div>
        </div>
        <div className="topic-inspector">
          <div className="inspector-heading">
            <span className="eyebrow">Message inspector</span>
            <span className="meta">Read only</span>
          </div>
          <label htmlFor="inspect-topic">Inspect a topic</label>
          <select
            id="inspect-topic"
            value={topic?.name ?? ""}
            onChange={(e) => {
              setSelected(e.target.value);
              setCopyStatus("");
            }}
          >
            <option value="">Select a topic</option>
            {sorted.map((t) => (
              <option key={t.name} value={t.name}>
                {t.name} · {t.type}
              </option>
            ))}
          </select>
          {selected ? (
            <>
              <div className="selected-topic">
                <code>{selected}</code>
                <small>{topic?.type ?? "Topic no longer available"}</small>
              </div>
              <StatusLine status={status} />
              <p className="meta delivery-rate">
                Browser-received rate:{" "}
                <strong>{hz?.toFixed(1) ?? "—"} Hz</strong> · throttled to ≤2
                Hz; not publisher frequency
              </p>
              <div className="inspector-actions">
                <button
                  disabled={!topic}
                  onClick={() => void copy(selected, "Topic name")}
                >
                  Copy topic name
                </button>
                <button
                  disabled={!sample.value}
                  onClick={() => void copy(sample.value!, "Displayed JSON")}
                >
                  Copy JSON
                </button>
              </div>
              <p role="status" className="meta copy-status">
                {copyStatus}
              </p>
              <pre tabIndex={0} aria-label="Latest topic message">
                {sample.value ?? "Waiting for the latest message."}
              </pre>
              <p className="meta source-note">
                Latest received message · output capped at 16,000 characters.
                Historical content is labelled by its receipt status above.
              </p>
            </>
          ) : (
            <EmptyState title="Select a topic to inspect">
              Browse the discovered topics to view the latest received JSON
              message. Output is capped at 16,000 characters.
            </EmptyState>
          )}
        </div>
      </div>
    </Panel>
  );
}
