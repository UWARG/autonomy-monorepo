import type { WidgetDefinition, WidgetProps } from '../types'

function TargetView({ data }: WidgetProps) {
  const target = (data.target ?? {}) as Record<string, unknown>
  const position = (data.position ?? {}) as Record<string, unknown>

  const tgtId = (target.id as string | undefined) ?? 'TGT-01'
  const tgtStatus = (target.status as string | undefined) ?? 'TRACKING'
  const distance = (target.distance as number | undefined) ?? '--'
  const bearing = (target.bearing as number | undefined) ?? '--'
  const tgtLat = (target.lat as number | undefined)?.toFixed(5) ?? '--'
  const tgtLon = (target.lon as number | undefined)?.toFixed(5) ?? '--'
  const tgtType = (target.type as string | undefined) ?? 'SURVEY POI'
  const droneLat = (position.lat as number | undefined)?.toFixed(5) ?? '--'
  const droneLon = (position.lon as number | undefined)?.toFixed(5) ?? '--'

  const statusColor = tgtStatus === 'TRACKING' ? 'var(--accent-green)' : 'var(--text-secondary)'

  return (
    <div className="target-widget">
      <div className="target-header-row">
        <span className="target-id">{tgtId}</span>
        <span className="target-status-badge" style={{ color: statusColor }}>{tgtStatus}</span>
      </div>

      <div className="target-metrics">
        <div className="target-metric">
          <span className="target-metric-label">DISTANCE</span>
          <span className="target-metric-val">{distance}<span className="target-metric-unit">m</span></span>
        </div>
        <div className="target-metric">
          <span className="target-metric-label">BEARING</span>
          <span className="target-metric-val">{String(typeof bearing === 'number' ? Math.round(bearing) : bearing).padStart(3, '0')}</span>
        </div>
      </div>

      <div className="target-section-label">TARGET POSITION</div>
      <div className="target-coord-row">
        <span className="target-coord-key">Lat</span>
        <span className="target-coord-val">N {tgtLat}°</span>
      </div>
      <div className="target-coord-row">
        <span className="target-coord-key">Lon</span>
        <span className="target-coord-val">W {tgtLon}°</span>
      </div>
      <div className="target-coord-row">
        <span className="target-coord-key">Type</span>
        <span className="target-coord-val">{tgtType}</span>
      </div>

      <div className="target-section-label">DRONE POSITION</div>
      <div className="target-coord-row">
        <span className="target-coord-key">Lat</span>
        <span className="target-coord-val">N {droneLat}°</span>
      </div>
      <div className="target-coord-row">
        <span className="target-coord-key">Lon</span>
        <span className="target-coord-val">W {droneLon}°</span>
      </div>
    </div>
  )
}

export const TargetWidget: WidgetDefinition = {
  id: 'target',
  label: 'Target',
  defaultLayout: { x: 10, y: 5, w: 2, h: 4, minW: 2, minH: 3 },
  subscriptions: ['target', 'position'],
  component: TargetView,
}
