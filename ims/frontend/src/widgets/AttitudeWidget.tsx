import type { WidgetDefinition, WidgetProps } from '../types'

function AttitudeView({ data }: WidgetProps) {
  const att = (data.attitude ?? {}) as Record<string, number>
  const tel = (data.telemetry ?? {}) as Record<string, number>

  const roll = att.roll ?? 0
  const pitch = att.pitch ?? 0
  const yaw = att.yaw ?? 0
  const gspd = tel.gspd ?? 0
  const aspd = tel.aspd ?? 0
  const vsi = tel.vsi ?? 0

  const pitchOffset = pitch * 4 * -1
  const rollDeg = (roll * 180) / Math.PI

  return (
    <div className="attitude-widget">
      <div className="attitude-horizon-wrap">
        <div className="attitude-horizon">
          <div
            className="attitude-ball"
            style={{
              transform: `rotate(${rollDeg}deg) translateY(${pitchOffset}px)`,
            }}
          />
          <div className="attitude-overlay">
            <div className="attitude-crosshair" />
            <div className="attitude-roll-indicator" style={{ transform: `rotate(${rollDeg}deg)` }}>
              <div className="attitude-roll-pointer" />
            </div>
          </div>
          <div className="attitude-pitch-scale">
            {[-20, -10, 0, 10, 20].map(v => (
              <div key={v} className={`pitch-line ${v === 0 ? 'pitch-line-zero' : ''}`} style={{ top: `${50 - v * 4}%` }}>
                <span>{v !== 0 ? Math.abs(v) : ''}</span>
              </div>
            ))}
          </div>
        </div>
      </div>

      <div className="attitude-readouts">
        <div className="att-val-group">
          <span className="att-label">ROLL</span>
          <span className="att-val">{roll >= 0 ? '+' : ''}{Math.round(roll)}</span>
        </div>
        <div className="att-val-group">
          <span className="att-label">PITCH</span>
          <span className="att-val">{pitch >= 0 ? '+' : ''}{Math.round(pitch)}</span>
        </div>
        <div className="att-val-group">
          <span className="att-label">YAW</span>
          <span className="att-val">{String(Math.round(yaw)).padStart(3, '0')}</span>
        </div>
      </div>

      <div className="attitude-speed-readouts">
        <div className="att-val-group">
          <span className="att-label">GSPD</span>
          <span className="att-val-lg">{gspd.toFixed(1)}<span className="att-unit">m/s</span></span>
        </div>
        <div className="att-val-group">
          <span className="att-label">ASPD</span>
          <span className="att-val-lg">{aspd.toFixed(1)}<span className="att-unit">m/s</span></span>
        </div>
        <div className="att-val-group">
          <span className="att-label">VSI</span>
          <span className={`att-val-lg ${vsi < 0 ? 'neg' : ''}`}>{vsi.toFixed(1)}<span className="att-unit">m/s</span></span>
        </div>
      </div>
    </div>
  )
}

export const AttitudeWidget: WidgetDefinition = {
  id: 'attitude',
  label: 'Attitude',
  defaultLayout: { x: 10, y: 0, w: 2, h: 5, minW: 2, minH: 4 },
  subscriptions: ['attitude', 'telemetry'],
  component: AttitudeView,
}
