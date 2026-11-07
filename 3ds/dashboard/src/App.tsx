import { useMemo, useState } from "react";
import { useConnection, useTopics, useNow } from "./hooks";
import { LiveTransport } from "./ros";
import { mockTransport } from "./mock";
import {
  TreePanel,
  MavrosPanel,
  PreflightPanel,
  TemperaturePanel,
} from "./components/Telemetry";
import { TopicsPanel } from "./components/Topics";
import { ImagesPanel } from "./components/Images";
const params = new URLSearchParams(window.location.search);
const isMock = params.get("mode") === "mock";
const defaultUrl = `${location.protocol === "https:" ? "wss" : "ws"}://${location.hostname || "localhost"}:9090`;
export default function App() {
  const [input, setInput] = useState(params.get("bridge") ?? defaultUrl);
  const [url, setUrl] = useState(input);
  const [urlError, setUrlError] = useState("");
  const [overview, setOverview] = useState<HTMLElement | null>(null);
  const transport = useMemo(
    () => (isMock ? mockTransport : new LiveTransport(url)),
    [url],
  );
  const connection = useConnection(transport),
    discovered = useTopics(transport, connection.state),
    now = useNow();
  const props = {
    transport,
    state: connection.state,
    topics: discovered.topics,
    now,
    preflight: params.get("preflight") ?? "/preflight/diagnostics",
    overview,
  };
  return (
    <>
      <header className="app-header">
        <div className="identity">
          <span className="wordmark">
            WARG<span> / 3DS</span>
          </span>
          <span className="header-rule" aria-hidden="true" />
          <div>
            <h1>3DS Drone Monitor</h1>
            <p className="eyebrow">Read-only monitoring</p>
          </div>
        </div>
        <div className="header-status">
          <span className={`badge ${isMock ? "mock" : ""}`}>
            {isMock ? "MOCK DATA" : "LIVE MODE"}
          </span>
          <span className={`connection-state ${connection.state}`}>
            <span aria-hidden="true" className="status-mark" />
            {connection.state === "connecting"
              ? "Connecting / retrying"
              : connection.state === "connected"
                ? "Connected"
                : "Disconnected"}
          </span>
        </div>
      </header>
      {isMock && (
        <aside className="mock-banner">
          MOCK MODE · All telemetry is synthetic. This does not indicate drone
          readiness. <a href="/">Switch to live</a>
        </aside>
      )}
      <details className="connection-settings">
        <summary>
          Connection Settings{" "}
          <span className="meta">
            {isMock ? "Synthetic demonstration" : "rosbridge endpoint"}
          </span>
        </summary>
        <div className="settings-content">
          {isMock ? (
            <p className="meta">
              Mock mode uses synthetic fixtures and does not connect to
              rosbridge. <a href="/">Switch to live</a>
            </p>
          ) : (
            <>
              <p className="endpoint">
                Active endpoint <code>{url}</code>
              </p>
              <form
                onSubmit={(e) => {
                  e.preventDefault();
                  try {
                    const parsed = new URL(input);
                    if (!["ws:", "wss:"].includes(parsed.protocol))
                      throw Error();
                    setUrl(input);
                    setUrlError("");
                  } catch {
                    setUrlError("Use a ws:// or wss:// URL");
                  }
                }}
              >
                <label>
                  rosbridge URL
                  <input
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    spellCheck={false}
                    autoComplete="off"
                    aria-describedby="connection-feedback"
                  />
                </label>
                <button type="submit">Connect</button>
                <a href="?mode=mock">Open mock demo</a>
              </form>
              <p id="connection-feedback" className="meta">
                {connection.state === "connected"
                  ? "Receiving from rosbridge. FCU availability is reported separately."
                  : connection.state === "connecting"
                    ? "Attempting connection. Automatic reconnection is enabled."
                    : "Bridge disconnected. Automatic retry will continue; check the endpoint and network."}
              </p>
            </>
          )}
        </div>
      </details>
      {(urlError || connection.error || discovered.error) && (
        <p role="alert" className="notice">
          {urlError || connection.error || discovered.error}
        </p>
      )}
      <main>
        <section aria-label="System overview" className="overview">
          <div className="overview-label">
            <span className="eyebrow">System overview</span>
            <span className="meta">
              {isMock ? "Synthetic sources" : "Current received state"}
            </span>
          </div>
          <dl className="overview-metrics" ref={setOverview} />
        </section>
        <div className="section-heading">
          <h2>Flight telemetry</h2>
          <p>Camera feed &amp; flight-controller state</p>
        </div>
        <div className="primary-grid">
          <ImagesPanel {...props} />
          <MavrosPanel {...props} />
        </div>
        <div className="section-heading">
          <h2>Mission &amp; system health</h2>
          <p>Engine activity, diagnostic results &amp; companion sensors</p>
        </div>
        <div className="secondary-grid">
          <TreePanel {...props} />
          <PreflightPanel {...props} />
          <TemperaturePanel {...props} />
        </div>
        <div className="section-heading">
          <h2>ROS workspace</h2>
          <p>Browse discovered topics &amp; inspect received messages</p>
        </div>
        <TopicsPanel {...props} />
      </main>
      <footer>
        <span className="eyebrow">WARG / AUTONOMY</span>
        <span>
          Receipt age uses the browser clock. Source timestamps use ROS time.
          Stale after 5 seconds. No flight controls.
        </span>
      </footer>
    </>
  );
}
