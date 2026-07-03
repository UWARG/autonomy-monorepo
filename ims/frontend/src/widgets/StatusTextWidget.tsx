import { useEffect, useState } from 'react';
import { subscribe, unsubscribe } from '../socket';
import './StatusTextWidget.css';

type TaskState = 'RUNNING' | 'SUCCESS' | 'FAILURE' | 'INVALID';

interface StatusPayload {
  task: string;
  state: TaskState;
  text: string;
}

interface StatusEntry extends StatusPayload {
  receivedAt: number;
}

const MAX_HISTORY = 5;

function StatusTextWidget() {
  const [history, setHistory] = useState<StatusEntry[]>([]);

  useEffect(() => {
    const handleStatus = (payload: StatusPayload) => {
      setHistory((prev) => [{ ...payload, receivedAt: Date.now() }, ...prev].slice(0, MAX_HISTORY));
    };

    subscribe<StatusPayload>('status', handleStatus);
    return () => unsubscribe<StatusPayload>('status', handleStatus);
  }, []);

  const [current, ...previous] = history;

  return (
    <div className="status-widget">
      <div className="status-widget__header">
        <h3>Task Status</h3>
        {current && (
          <span className={`status-badge status-badge--${current.state}`}>
            {current.state}
          </span>
        )}
      </div>

      {current ? (
        <>
          <div className="status-widget__task">{current.task}</div>
          <p className="status-widget__text">{current.text}</p>
        </>
      ) : (
        <p className="status-widget__empty">Waiting for status updates…</p>
      )}

      {previous.length > 0 && (
        <ul className="status-widget__history">
          {previous.map((entry) => (
            <li key={entry.receivedAt}>
              <span className={`status-dot status-dot--${entry.state}`} />
              <span className="status-widget__history-task">{entry.task}</span>
              <span className="status-widget__history-text">{entry.text}</span>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

export default StatusTextWidget;
