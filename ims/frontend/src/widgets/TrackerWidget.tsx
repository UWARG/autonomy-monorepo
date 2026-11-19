import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import mqtt from 'mqtt';
import { ros } from '../ros.js';

const ROS_TOPIC = '/tracker/location';
const MQTT_URL = 'ws://broker.emqx.io:8083/mqtt';
const MQTT_TOPIC = 'warg/efs-lte-tracker/+/location';
const STORAGE_PREFIX = 'tracker-fixes:';
const STALE_MS = 5000; // no fix for 5 s = stale
const DASH = '\u2014';

interface Fix { t: number; lat: number; lon: number } // t = GPS time, ms since epoch
type Sessions = Record<string, Fix[]>;              // session id -> fixes sorted by t

interface RosNavSatFix {
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  latitude: number;
  longitude: number;
}

type DirPicker = (o?: { mode?: 'read' | 'readwrite' }) => Promise<FileSystemDirectoryHandle>;
const showDirectoryPicker = (window as unknown as { showDirectoryPicker?: DirPicker }).showDirectoryPicker;

function addFix(prev: Sessions, session: string, fix: Fix): Sessions {
  const list = prev[session] ?? [];
  if (list.some((f) => f.t === fix.t)) return prev; // already have it (other source or QoS 1 repeat)
  return { ...prev, [session]: [...list, fix].sort((a, b) => a.t - b.t) };
}

function toCsv(fixes: Fix[]): string {
  const rows = fixes.map((f) => `${new Date(f.t).toISOString()},${f.lat.toFixed(7)},${f.lon.toFixed(7)}`);
  return ['timestamp,latitude,longitude', ...rows].join('\n') + '\n';
}

const fileName = (session: string) => `tracker_${session.replace(/[^A-Za-z0-9_-]/g, '_')}.csv`;

function loadSaved(): Sessions {
  const out: Sessions = {};
  try {
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key?.startsWith(STORAGE_PREFIX)) {
        out[key.slice(STORAGE_PREFIX.length)] = JSON.parse(localStorage.getItem(key) ?? '[]');
      }
    }
  } catch { /* storage unavailable: start empty */ }
  return out;
}

// metres east/north of origin (fine for a field-sized area)
function enu(o: Fix, p: Fix) {
  const R = 6_371_000, rad = Math.PI / 180;
  return {
    east: (p.lon - o.lon) * rad * Math.cos(o.lat * rad) * R,
    north: (p.lat - o.lat) * rad * R,
  };
}

const VIEW = 200;
const CENTER = VIEW / 2;

export default function TrackerWidget() {
  const [sessions, setSessions] = useState<Sessions>(loadSaved);
  const [picked, setPicked] = useState<string>();
  const [rosLastRx, setRosLastRx] = useState(0);
  const [mqttUp, setMqttUp] = useState(false);
  const [dir, setDir] = useState<FileSystemDirectoryHandle | null>(null);
  const [saveError, setSaveError] = useState<string>();
  const [now, setNow] = useState(() => Date.now());

  // tick once a second so "last fix Xs ago" updates
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

  // Source 1: ROS topic from the airside tracker_bridge (via rosbridge)
  useEffect(() => {
    const topic = new ROSLIB.Topic<RosNavSatFix>({ ros, name: ROS_TOPIC, messageType: 'sensor_msgs/NavSatFix' });
    const onFix = (m: RosNavSatFix) => {
      const t = Math.round(m.header.stamp.sec * 1000 + m.header.stamp.nanosec / 1e6);
      setSessions((prev) => addFix(prev, m.header.frame_id, { t, lat: m.latitude, lon: m.longitude }));
      setRosLastRx(Date.now());
    };
    topic.subscribe(onFix);
    return () => topic.unsubscribe(onFix);
  }, []);

  // Source 2: straight from the MQTT broker (works even if the drone link is down)
  useEffect(() => {
    const client = mqtt.connect(MQTT_URL, { reconnectPeriod: 3000 });
    client.on('connect', () => {
      setMqttUp(true);
      client.subscribe(MQTT_TOPIC, { qos: 1 });
    });
    client.on('close', () => setMqttUp(false));
    client.on('message', (topic, payload) => {
      try {
        const d = JSON.parse(payload.toString());
        const session = topic.split('/')[2];
        if (d.session_id !== session) return;                       // malformed / spoofed
        if (!d.fix || d.latitude == null || d.longitude == null || !d.gnss_time) return; // no GPS lock
        if (d.sequence <= 0) return;                                // debug-page test sample
        const t = Date.parse(d.gnss_time);
        if (Number.isNaN(t)) return;
        setSessions((prev) => addFix(prev, session, { t, lat: d.latitude, lon: d.longitude }));
      } catch { /* ignore non-JSON */ }
    });
    return () => { client.end(true); };
  }, []);

  // Backup 1: browser storage (survives a page refresh)
  useEffect(() => {
    try {
      for (const [s, fixes] of Object.entries(sessions)) {
        localStorage.setItem(STORAGE_PREFIX + s, JSON.stringify(fixes));
      }
    } catch { /* storage full or blocked */ }
  }, [sessions]);

  // Backup 3: auto-save one CSV per session into the chosen folder
  useEffect(() => {
    if (!dir) return;
    (async () => {
      try {
        for (const [s, fixes] of Object.entries(sessions)) {
          const file = await dir.getFileHandle(fileName(s), { create: true });
          const w = await file.createWritable();
          await w.write(toCsv(fixes));
          await w.close();
        }
        setSaveError(undefined);
      } catch (e) {
        setSaveError(e instanceof Error ? e.message : 'Auto-save failed');
      }
    })();
  }, [sessions, dir]);

  // default to the session with the most recent fix
  const ids = Object.keys(sessions).sort(
    (a, b) => (sessions[b].at(-1)?.t ?? 0) - (sessions[a].at(-1)?.t ?? 0),
  );
  const selected = picked && sessions[picked] ? picked : ids[0];
  const fixes = selected ? sessions[selected] : [];
  const latest = fixes.at(-1);
  const ageMs = latest ? now - latest.t : null;
  const stale = ageMs == null || ageMs > STALE_MS;
  const rosUp = now - rosLastRx < STALE_MS;

  // Backup 2: manual download
  const download = () => {
    if (!selected) return;
    const url = URL.createObjectURL(new Blob([toCsv(fixes)], { type: 'text/csv' }));
    const a = document.createElement('a');
    a.href = url;
    a.download = fileName(selected);
    a.click();
    URL.revokeObjectURL(url);
  };

  // plot: path relative to first fix, auto-scaled
  const origin = fixes[0];
  const pts = origin ? fixes.map((f) => enu(origin, f)) : [];
  const extent = Math.max(20, ...pts.map((p) => Math.max(Math.abs(p.east), Math.abs(p.north))));
  const scale = (CENTER - 12) / extent;
  const xy = pts.map((p) => ({ x: CENTER + p.east * scale, y: CENTER - p.north * scale }));

  const pill = !latest
    ? { className: 'pill bg-edge text-ink-3', label: 'NO DATA' }
    : stale
      ? { className: 'pill-warn', label: 'STALE' }
      : { className: 'pill-ok', label: 'LIVE' };

  return (
    <section className="widget flex h-full min-h-[120px] flex-col p-4">
      <header className="flex items-center justify-between gap-2">
        <h2 className="widget-label">Tracker</h2>
        <select
          aria-label="Tracker session"
          value={selected ?? ''}
          onChange={(e) => setPicked(e.target.value)}
          className="max-w-[140px] truncate rounded bg-feed p-1 font-mono text-[11px] text-ink-2"
        >
          {ids.length === 0 && <option value="">no sessions</option>}
          {ids.map((s) => <option key={s} value={s}>{s.slice(0, 8)} ({sessions[s].length})</option>)}
        </select>
        <span className={pill.className}>{pill.label}</span>
      </header>

      <div className="relative mt-3 min-h-0 flex-1 overflow-hidden rounded-lg bg-feed">
        <svg viewBox={`0 0 ${VIEW} ${VIEW}`} className="h-full w-full">
          {xy.length > 1 && (
            <polyline points={xy.map((p) => `${p.x},${p.y}`).join(' ')} fill="none" stroke="#3B6FD4" strokeWidth="1.5" />
          )}
          {xy.length > 0 && <circle cx={xy.at(-1)!.x} cy={xy.at(-1)!.y} r="4" fill="#34D17E" />}
          {xy.length === 0 && (
            <text x={CENTER} y={CENTER} textAnchor="middle" fontSize="10" fill="#6B7688">Waiting for tracker</text>
          )}
          <text x="6" y={VIEW - 6} fontSize="7" fill="#6B7688">{Math.round(extent)} m</text>
        </svg>
      </div>

      <footer className="mt-3 flex flex-col gap-2 text-[11px]">
        <div className="flex justify-between font-mono text-ink-2">
          <span>{latest ? `${latest.lat.toFixed(6)}, ${latest.lon.toFixed(6)}` : DASH}</span>
          <span>{ageMs != null ? `${Math.max(0, Math.round(ageMs / 1000))}s ago` : DASH}</span>
        </div>
        <div className="flex items-center gap-3">
          <span className="flex items-center gap-1.5"><span className={`status-dot ${rosUp ? 'bg-ok' : 'bg-bad'}`} />ROS</span>
          <span className="flex items-center gap-1.5"><span className={`status-dot ${mqttUp ? 'bg-ok' : 'bg-bad'}`} />MQTT</span>
          <button onClick={download} disabled={!selected} className="ml-auto rounded bg-edge px-2 py-1 text-ink-2">
            Download CSV
          </button>
          <button
            onClick={async () => setDir(await showDirectoryPicker!({ mode: 'readwrite' }))}
            disabled={!showDirectoryPicker}
            title={showDirectoryPicker ? 'Keep a CSV per session in a folder' : 'Needs Chrome or Edge'}
            className="rounded bg-edge px-2 py-1 text-ink-2"
          >
            {dir ? `Saving → ${dir.name}` : 'Auto-save folder'}
          </button>
        </div>
        {saveError && <p role="alert" className="text-bad">{saveError}</p>}
      </footer>
    </section>
  );
}