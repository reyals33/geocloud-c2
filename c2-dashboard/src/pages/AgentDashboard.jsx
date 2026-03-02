import { lazy, Suspense } from 'react'
import VideoFeed        from '../components/VideoFeed'
import TelemetryPanel   from '../components/TelemetryPanel'
import ControlPanel     from '../components/ControlPanel'
import { useAgent }     from '../hooks/useAgent'

// Lazy-load MiniMap so it doesn't block the main dashboard render
const MiniMap = lazy(() => import('../components/MiniMap'))

export default function AgentDashboard({ agentId }) {
  const { pose, rtt, connected } = useAgent(agentId)

  return (
    <div className="dashboard-body">
      {/* Main panel: first-person camera feed — relative so MiniMap overlays */}
      <div className="scene-panel" style={{ position: 'relative' }}>
        <VideoFeed agentId={agentId} />

        {/* Mini-map: Three.js positional overlay — bottom-right corner */}
        <Suspense fallback={null}>
          <MiniMap currentAgent={agentId} />
        </Suspense>
      </div>

      <div className="side-col">
        <TelemetryPanel
          agentId={agentId}
          pose={pose}
          rtt={rtt}
          connected={connected}
        />
        <ControlPanel agentId={agentId} />
      </div>
    </div>
  )
}
