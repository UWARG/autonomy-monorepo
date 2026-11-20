import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { ros } from '../ros.js';

interface Diagnostic {
  name: string;
  level: number;
  message: string;
  values?: { key: string; value: string }[];
}

interface PreflightStatus {
  ready: boolean;
  message: string;
  checks: Diagnostic[];
}

const waiting: PreflightStatus = {
  ready: false,
  message: 'Waiting for drone diagnostics',
  checks: [],
};

export default function PreflightWidget() {
  const [status, setStatus] = useState(waiting);

  useEffect(function () {
    const heartbeat = new ROSLIB.Topic({
      ros, name: '/heartbeat', messageType: 'std_msgs/msg/Empty',
    });
    const reply = new ROSLIB.Topic({
      ros, name: '/groundstation/heartbeat', messageType: 'std_msgs/msg/Empty',
    });
    const diagnostics = new ROSLIB.Topic<{ status: Diagnostic[] }>({
      ros, name: '/diagnostics', messageType: 'diagnostic_msgs/msg/DiagnosticArray',
    });
    let lastHeartbeat = -Infinity;
    let lastDiagnostics = -Infinity;
    let lastReply = -Infinity;
    let latest = waiting;

    function onHeartbeat() {
      lastHeartbeat = performance.now();
    }

    function onDiagnostics(message: { status: Diagnostic[] }) {
      if (!Array.isArray(message.status)) return;
      const summary = message.status.find(function (check) {
        return check?.name === 'preflight_node: readiness';
      });
      // Other ROS nodes publish separate arrays on this same topic.
      if (!summary) return;
      lastDiagnostics = performance.now();
      const droneReady = summary.level === 0 && Array.isArray(summary.values) &&
        summary.values.some(function (entry) {
          return entry?.key === 'ready' && entry.value === 'true';
        });
      latest = {
        ready: droneReady,
        message: summary.level === 0 && !droneReady ? 'Readiness not confirmed by drone' :
          typeof summary.message === 'string' ? summary.message : 'Invalid diagnostics',
        checks: message.status.filter(function (check) {
          return typeof check?.name === 'string' &&
            check.name.startsWith('preflight_node: ') && check !== summary;
        }),
      };
    }

    function reset() {
      lastHeartbeat = -Infinity;
      lastDiagnostics = -Infinity;
      lastReply = -Infinity;
      latest = waiting;
      setStatus(waiting);
    }

    function update() {
      const now = performance.now();
      const heartbeatFresh = now - lastHeartbeat < 3000;
      const diagnosticsFresh = now - lastDiagnostics < 3000;
      if (ros.isConnected && heartbeatFresh && now - lastReply >= 1000) {
        reply.publish(new ROSLIB.Message({}));
        lastReply = now;
      }
      let next = latest;
      if (!ros.isConnected) {
        next = { ready: false, message: 'Drone connection lost', checks: waiting.checks };
      } else if (!heartbeatFresh || !diagnosticsFresh) {
        next = {
          ready: false,
          message: !heartbeatFresh ? 'Waiting for drone heartbeat' : 'Diagnostics missing or stale',
          checks: waiting.checks,
        };
      }
      setStatus(function (previous) {
        return previous.ready === next.ready && previous.message === next.message &&
          previous.checks === next.checks ? previous : next;
      });
    }

    heartbeat.subscribe(onHeartbeat);
    diagnostics.subscribe(onDiagnostics);
    ros.on('connection', reset);
    ros.on('close', reset);
    ros.on('error', reset);
    const timer = setInterval(update, 100);
    return function () {
      clearInterval(timer);
      heartbeat.unsubscribe(onHeartbeat);
      diagnostics.unsubscribe(onDiagnostics);
      reply.unadvertise();
      ros.off('connection', reset);
      ros.off('close', reset);
      ros.off('error', reset);
    };
  }, []);

  return (
    <section className="widget p-4" aria-label="Preflight checks">
      <header className="flex items-center justify-between gap-4">
        <h2 className="widget-label">Preflight</h2>
        <span className={status.ready ? 'pill-ok' : 'pill-bad'}>
          {status.ready ? 'READY FOR TAKEOFF' : 'NOT READY'}
        </span>
      </header>
      <p className="mt-2 text-[13px] text-ink-2" aria-live="polite">{status.message}</p>
      {status.checks.length > 0 && (
        <details className="mt-2 text-[13px]">
          <summary className="cursor-pointer text-ink-3">Check details</summary>
          <ul className="mt-2 max-h-48 space-y-1 overflow-y-auto">
            {status.checks.map(function (check) {
              return (
                <li key={check.name}>
                  <span className={check.level === 0 ? 'text-green-400' : 'text-red-400'}>
                    {check.level === 0 ? 'PASS' : 'FAIL'}
                  </span>{' '}
                  {check.name.replace('preflight_node: ', '')}: {check.message}
                </li>
              );
            })}
          </ul>
        </details>
      )}
    </section>
  );
}
