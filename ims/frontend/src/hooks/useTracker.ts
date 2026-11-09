import { useEffect, useState } from 'react';
import mqtt from 'mqtt';

// Tracker fixes straight from the MQTT broker (works even if the drone link is down),
// kept per session in browser storage and optionally auto-saved as CSV.

const MQTT_URL = 'ws://broker.emqx.io:8083/mqtt';
const MQTT_TOPIC = 'warg/efs-lte-tracker/+/location';
const STORAGE_PREFIX = 'tracker-fixes:';
const STALE_MS = 5000; // no fix for 5 s = stale

export interface Fix { t: number; lat: number; lon: number } // t = GPS time, ms since epoch
type Sessions = Record<string, Fix[]>;              // session id -> fixes sorted by t

type DirPicker = (o?: { mode?: 'read' | 'readwrite' }) => Promise<FileSystemDirectoryHandle>;
const showDirectoryPicker = (window as unknown as { showDirectoryPicker?: DirPicker }).showDirectoryPicker;

function addFix(prev: Sessions, session: string, fix: Fix): Sessions {
  const list = prev[session] ?? [];
  if (list.some((f) => f.t === fix.t)) return prev; // already have it (QoS 1 repeat)
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

export type TrackerState = ReturnType<typeof useTracker>;

export default function useTracker() {
  const [sessions, setSessions] = useState<Sessions>(loadSaved);
  const [picked, setPicked] = useState<string>();
  const [mqttUp, setMqttUp] = useState(false);
  const [dir, setDir] = useState<FileSystemDirectoryHandle | null>(null);
  const [saveError, setSaveError] = useState<string>();
  const [now, setNow] = useState(() => Date.now());

  // tick once a second so "last fix Xs ago" updates
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

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

  // Wipe the selected session: on screen, in browser storage, and its CSV in the auto-save folder
  const reset = async () => {
    if (!selected) return;
    if (!window.confirm(`Reset session ${selected.slice(0, 8)}? This deletes its ${fixes.length} fixes and its CSV file. This cannot be undone.`)) return;
    setSessions((prev) => {
      const next = { ...prev };
      delete next[selected];
      return next;
    });
    setPicked(undefined);
    try { localStorage.removeItem(STORAGE_PREFIX + selected); } catch { /* storage unavailable */ }
    if (!dir) return;
    try {
      await dir.removeEntry(fileName(selected)).catch((e) => {
        if (!(e instanceof DOMException && e.name === 'NotFoundError')) throw e;
      });
      setSaveError(undefined);
    } catch (e) {
      setSaveError(e instanceof Error ? e.message : 'Could not delete CSV file');
    }
  };

  const pickDir = async () => { if (showDirectoryPicker) setDir(await showDirectoryPicker({ mode: 'readwrite' })); };

  return {
    sessions, ids, selected, setPicked, fixes, latest, ageMs, stale, mqttUp,
    dir, pickDir, canPickDir: !!showDirectoryPicker, saveError, download, reset,
  };
}
