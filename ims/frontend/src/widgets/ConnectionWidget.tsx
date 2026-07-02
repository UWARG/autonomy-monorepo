import type { WidgetDefinition, WidgetProps } from '../types'

function ConnectionView({ data, send }: WidgetProps) {
  const conn = (data.connection ?? {}) as Record<string, unknown>

  const isLive = conn.link_active as boolean | undefined
  const protocol = (conn.protocol as string | undefined) ?? 'MAVLink 2.0'
  const transport = (conn.transport as string | undefined) ?? 'WebSocket :8765'
  const heartbeat = (conn.heartbeat_hz as number | undefined)?.toFixed(1) ?? '--'
  const latency = (conn.latency_ms as number | undefined) ?? '--'
  const packetLoss = (conn.packet_loss as number | undefined)?.toFixed(1) ?? '--'
  const msgRate = (conn.msg_rate as number | undefined) ?? '--'

  const lossNum = parseFloat(String(packetLoss))
  const lossColor = lossNum > 5 ? 'var(--accent-red)' : lossNum > 1 ? 'var(--accent-orange)' : 'var(--accent-green)'

  return (
    <div className="connection-widget">
      <div
        className={`link-status-card ${isLive ? 'link-active' : 'link-inactive'}`}
        onClick={() => send('simulate_link_drop')}
        title="Click to simulate link drop"
      >
        <div className="link-status-left">
          <span className={`dot ${isLive ? 'dot-green' : 'dot-red'}`} style={{ width: 10, height: 10 }} />
          <div>
            <div className="link-status-label">{isLive ? 'LINK ACTIVE' : 'LINK OFFLINE'}</div>
            <div className="link-status-sub">MAVLink heartbeat {isLive ? 'nominal' : 'lost'}</div>
          </div>
        </div>
      </div>

      <table className="conn-table">
        <tbody>
          <tr><td className="conn-key">Protocol</td><td className="conn-val">{protocol}</td></tr>
          <tr><td className="conn-key">Transport</td><td className="conn-val">{transport}</td></tr>
          <tr><td className="conn-key">Heartbeat</td><td className="conn-val">{heartbeat} Hz</td></tr>
          <tr><td className="conn-key">Latency</td><td className="conn-val">{latency} ms</td></tr>
          <tr>
            <td className="conn-key">Packet loss</td>
            <td className="conn-val" style={{ color: lossColor }}>{packetLoss} %</td>
          </tr>
          <tr><td className="conn-key">Msg rate</td><td className="conn-val">{msgRate} /s</td></tr>
        </tbody>
      </table>

      <div className="conn-hint">click card to simulate link drop</div>
    </div>
  )
}

export const ConnectionWidget: WidgetDefinition = {
  id: 'connection',
  label: 'Connection',
  defaultLayout: { x: 6, y: 5, w: 4, h: 4, minW: 2, minH: 3 },
  subscriptions: ['connection'],
  component: ConnectionView,
}
