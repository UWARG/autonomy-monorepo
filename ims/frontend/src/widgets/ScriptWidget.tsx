import type { WidgetDefinition, WidgetProps } from '../types'

function ScriptView({ data, send }: WidgetProps) {
  const mission = (data.mission ?? {}) as Record<string, unknown>

  const scriptName = (mission.script_name as string | undefined) ?? 'survey_grid.py'
  const status = (mission.status as string | undefined) ?? 'IDLE'
  const wpCurrent = (mission.wp_current as number | undefined) ?? 0
  const wpTotal = (mission.wp_total as number | undefined) ?? 8
  const wpStatus = (mission.wp_status as string | undefined) ?? 'IDLE'
  const progress = wpTotal > 0 ? (wpCurrent / wpTotal) * 100 : 0
  const isRunning = status === 'RUNNING'
  const isPaused = status === 'PAUSED'

  return (
    <div className="script-widget">
      <div className="script-header-row">
        <div className="script-name-row">
          <span className="script-name">{scriptName}</span>
        </div>
        <span className={`script-status-badge ${isRunning ? 'badge-green' : isPaused ? 'badge-orange' : 'badge-gray'}`}>
          {status}
        </span>
      </div>

      <div className="script-wp-row">
        WP {wpCurrent}/{wpTotal} — {wpStatus}
      </div>

      <div className="script-progress-track">
        <div className="script-progress-fill" style={{ width: `${progress}%` }} />
      </div>

      <div className="script-controls">
        <button
          className="script-btn btn-pause"
          disabled={!isRunning}
          onClick={() => send('mission_pause')}
        >
          ‖ Pause
        </button>
        <button
          className="script-btn btn-resume"
          disabled={!isPaused}
          onClick={() => send('mission_resume')}
        >
          ▷ Resume
        </button>
        <button
          className="script-btn btn-abort"
          disabled={status === 'IDLE'}
          onClick={() => send('mission_abort')}
        >
          ■ Abort
        </button>
      </div>
    </div>
  )
}

export const ScriptWidget: WidgetDefinition = {
  id: 'script',
  label: 'Mission Script',
  defaultLayout: { x: 0, y: 9, w: 6, h: 4, minW: 3, minH: 3 },
  subscriptions: ['mission'],
  actions: [
    { id: 'abort', label: 'Abort', handler: send => send('mission_abort') },
  ],
  component: ScriptView,
}
