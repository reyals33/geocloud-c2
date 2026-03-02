import { AGENT_MAP } from '../config'

function fmt(val, dec = 2) {
  return val != null ? val.toFixed(dec) : '--'
}

function rttClass(rtt) {
  if (rtt == null) return ''
  if (rtt < 100)  return 'rtt-value'
  if (rtt < 250)  return 'rtt-value warn'
  return 'rtt-value bad'
}

export default function TelemetryPanel({ agentId, pose, rtt, connected }) {
  const agent = AGENT_MAP[agentId]
  const yawDeg = pose ? (pose.yaw * 180 / Math.PI).toFixed(1) : '--'

  return (
    <div className="card">
      <div className="status-row">
        <div className={`status-dot ${connected ? 'live' : ''}`} />
        <span className={`status-label ${connected ? 'live' : ''}`}>
          {connected ? 'Live' : 'Offline'}
        </span>
        <span className="agent-name" style={{ color: agent.color }}>
          {agent.label}
        </span>
      </div>

      <div className="card-title">Telemetry</div>
      <div className="telem-grid">
        <div className="telem-item">
          <div className="telem-label">X</div>
          <div className="telem-value">{fmt(pose?.x)}<span className="telem-unit">m</span></div>
        </div>
        <div className="telem-item">
          <div className="telem-label">Y</div>
          <div className="telem-value">{fmt(pose?.y)}<span className="telem-unit">m</span></div>
        </div>
        {agent.type === 'drone' && (
          <div className="telem-item">
            <div className="telem-label">Alt</div>
            <div className="telem-value">{fmt(pose?.z)}<span className="telem-unit">m</span></div>
          </div>
        )}
        <div className="telem-item">
          <div className="telem-label">Heading</div>
          <div className="telem-value">{yawDeg}<span className="telem-unit">°</span></div>
        </div>
        <div className="telem-item" style={{ gridColumn: '1 / -1' }}>
          <div className="telem-label">Net RTT <span style={{ fontWeight: 400, opacity: 0.55, fontSize: 9 }}>WS ping</span></div>
          <div className={`telem-value ${rttClass(rtt)}`}>
            {rtt != null ? rtt : '--'}
            <span className="telem-unit">ms</span>
          </div>
        </div>
      </div>
    </div>
  )
}
