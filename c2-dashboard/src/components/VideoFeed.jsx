/**
 * VideoFeed — live robot camera panel.
 *
 * Streams via WebRTC (useWebRTC hook) from the GStreamer producer on EC2.
 * Rosbridge stays intact for all other topics (odom, cmd_vel, etc).
 *
 * Shows a placeholder when the WebRTC stream is not yet connected.
 * Tracks and displays the live frame rate (fps) via requestVideoFrameCallback.
 * Drone 2 only: "ENHANCED VISION" WebGL post-processing overlay.
 */

import { useState } from 'react'
import { useWebRTC }          from '../hooks/useWebRTC'
import { AGENT_MAP, WEBRTC_URL } from '../config'
import EnhancedVision         from './EnhancedVision'

export default function VideoFeed({ agentId }) {
  const agent      = AGENT_MAP[agentId]
  const hasEV      = !!agent.enhancedVision

  const { videoRef, connected, fps } = useWebRTC(agentId)

  const [enhanced, setEnhanced] = useState(false)
  const [playing,  setPlaying]  = useState(false)

  const hasVideo = connected && playing

  return (
    <div className="video-feed-wrap">
      {/* Live video — always in DOM so videoRef is stable */}
      <video
        ref={videoRef}
        autoPlay
        muted
        playsInline
        className="video-feed-img"
        style={{ display: hasVideo ? 'block' : 'none' }}
        onPlaying={() => setPlaying(true)}
        onPause={()   => setPlaying(false)}
        onEnded={()   => setPlaying(false)}
      />

      {/* Enhanced Vision WebGL overlay — drone2 only, when active */}
      {hasEV && enhanced && hasVideo && (
        <EnhancedVision mediaRef={videoRef} />
      )}

      {/* Placeholder — shown until WebRTC stream is live */}
      {!hasVideo && (
        <div className="video-feed-placeholder">
          <div className="vfp-icon">⬛</div>
          <div className="vfp-topic">WebRTC · /{agentId}/camera</div>
          <div className="vfp-status">AWAITING VIDEO STREAM</div>
          <div className="vfp-hint">
            Connecting to GStreamer producer on EC2<br />
            via WebRTC signaling server (port {new URL(WEBRTC_URL).port})
          </div>
          <div className="vfp-agent" style={{ color: agent.color }}>
            {agent.label} — {agent.type === 'drone' ? 'Drone FPV' : 'Ground Robot Camera'}
          </div>
        </div>
      )}

      {/* Enhanced Vision toggle — top-right, drone2 only */}
      {hasEV && (
        <button
          className={`ev-btn${enhanced ? ' active' : ''}`}
          onClick={() => setEnhanced(v => !v)}
        >
          {enhanced ? 'ENHANCED VISION ✦' : 'ENHANCED VISION'}
        </button>
      )}

      {/* Status overlay — bottom of video */}
      <div className="video-feed-label" style={{ borderColor: agent.color }}>
        <span className="vfl-dot" style={{ background: hasVideo ? '#22c55e' : '#475569' }} />
        {hasVideo ? 'LIVE' : 'NO SIGNAL'} &nbsp;·&nbsp; {agent.label} &nbsp;·&nbsp; {agent.width}×{agent.height}
        {hasVideo && fps != null && (
          <span style={{ marginLeft: 8, opacity: 0.7 }}>· {fps} fps</span>
        )}
      </div>
    </div>
  )
}
