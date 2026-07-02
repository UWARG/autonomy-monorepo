import { useEffect, useRef, useState } from 'react'
import type { WidgetDefinition, WidgetProps } from '../types'
import { subscribe, unsubscribe } from '../socket'

interface LogEntry {
  time: string
  level: 'INF' | 'WRN' | 'ERR'
  msg: string
}

function LogView(_props: WidgetProps) {
  const [entries, setEntries] = useState<LogEntry[]>([])
  const bottomRef = useRef<HTMLDivElement>(null)

  useEffect(() => {
    const handler = (payload: unknown) => {
      const entry = payload as LogEntry
      setEntries(prev => [...prev.slice(-199), entry])
    }
    subscribe('log', handler)
    return () => unsubscribe('log', handler)
  }, [])

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [entries])

  const levelColor = (level: string) => {
    if (level === 'WRN') return 'var(--accent-orange)'
    if (level === 'ERR') return 'var(--accent-red)'
    return 'var(--accent-blue)'
  }

  return (
    <div className="log-widget">
      {entries.length === 0 ? (
        <span className="log-empty">No log entries yet</span>
      ) : (
        entries.map((e, i) => (
          <div key={i} className="log-entry">
            <span className="log-time">{e.time}</span>
            <span className="log-level" style={{ color: levelColor(e.level) }}>{e.level}</span>
            <span className="log-msg">{e.msg}</span>
          </div>
        ))
      )}
      <div ref={bottomRef} />
    </div>
  )
}

export const LogWidget: WidgetDefinition = {
  id: 'log',
  label: 'Telemetry Log',
  defaultLayout: { x: 6, y: 9, w: 6, h: 4, minW: 3, minH: 2 },
  subscriptions: ['log'],
  component: LogView,
}
