import { useWidgetData } from './useWidgetData'

export function StatusBar() {
  const data = useWidgetData(['telemetry', 'connection', 'gps'])

  const tel = (data.telemetry ?? {}) as Record<string, unknown>
  const conn = (data.connection ?? {}) as Record<string, unknown>
  const gps = (data.gps ?? {}) as Record<string, unknown>

  const batt = (tel.battery_pct as number | undefined) ?? '--'
  const alt = (tel.alt as number | undefined)?.toFixed(1) ?? '--'
  const gspd = (tel.gspd as number | undefined)?.toFixed(1) ?? '--'
  const sats = (gps.sats as number | undefined) ?? '--'
  const linkLoss = (conn.packet_loss as number | undefined)?.toFixed(1) ?? '--'
  const isLive = conn.link_active as boolean | undefined

  const now = new Date()
  const time = now.toTimeString().slice(0, 8)

  return (
    <header className="status-bar">
      <div className="status-bar-left">
        <span className="ims-logo">IMS</span>
        <span className="ims-full">Intelligent Monitoring System</span>
        <span className="status-pill pill-gray">SIM MODE</span>
        <span className="status-pill pill-orange">
          <span className="dot dot-red" /> LIVE FLIGHT
        </span>
        <span className={`status-pill ${isLive ? 'pill-blue' : 'pill-gray'}`}>
          LINK {isLive ? 'LIVE' : 'OFFLINE'}
        </span>
        {linkLoss !== '--' && (
          <span className="status-loss">{linkLoss}% loss</span>
        )}
        <span className="status-pill pill-gray">AUTO</span>
      </div>

      <div className="status-bar-right">
        <span className="tel-item"><span className="tel-label">BATT</span>{batt}%</span>
        <span className="tel-item"><span className="tel-label">ALT</span>{alt}<span className="tel-unit">m</span></span>
        <span className="tel-item"><span className="tel-label">GSPD</span>{gspd}<span className="tel-unit">m/s</span></span>
        <span className="tel-item"><span className="tel-label">GPS</span>{sats}</span>
        <span className="tel-clock">{time}</span>
      </div>
    </header>
  )
}
