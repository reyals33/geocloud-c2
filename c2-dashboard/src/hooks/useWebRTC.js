/**
 * useWebRTC — connects to the signaling server as a consumer,
 * receives the WebRTC offer from the GStreamer producer,
 * and returns a ref to the <video> element playing the live stream.
 *
 * Trickle ICE strategy (matches GStreamer webrtcbin's default behaviour):
 *   - Answer is sent immediately after setLocalDescription
 *   - Each gathered ICE candidate is forwarded individually via signaling
 *   - Incoming ICE candidates from the producer are applied via addIceCandidate
 *
 * Reconnects automatically on signaling WebSocket close.
 */

import { useEffect, useRef, useState } from 'react'
import { WEBRTC_URL } from '../config'

export function useWebRTC(agentId) {
  const videoRef  = useRef(null)
  const pcRef     = useRef(null)
  const wsRef     = useRef(null)

  const [connected, setConnected] = useState(false)
  const [fps,       setFps]       = useState(null)

  // FPS tracking via requestVideoFrameCallback (Chrome 83+, Firefox 130+)
  const frameCountRef = useRef(0)
  const fpsWindowRef  = useRef(performance.now())
  const fpsGenRef     = useRef(0)  // incremented on each new track; stops stale rVFC chains

  useEffect(() => {
    let closed = false

    const closePc = () => {
      if (pcRef.current) {
        pcRef.current.close()
        pcRef.current = null
      }
      setConnected(false)
    }

    const connect = () => {
      const ws = new WebSocket(WEBRTC_URL)
      wsRef.current = ws

      ws.onopen = () => {
        ws.send(JSON.stringify({ type: 'join', agent: agentId }))
      }

      ws.onmessage = async (ev) => {
        let msg
        try { msg = JSON.parse(ev.data) } catch { return }

        // ── Incoming offer (from GStreamer producer) ──────────────────────────
        if (msg.type === 'offer') {
          // Close any stale peer connection before creating a new one
          closePc()

          const pc = new RTCPeerConnection({
            iceServers: [{ urls: 'stun:stun.l.google.com:19302' }],
          })
          pcRef.current = pc

          // ── Trickle ICE: send each local candidate as it is gathered ────────
          pc.onicecandidate = (e) => {
            if (!e.candidate) return  // null = gathering complete; nothing to send
            ws.send(JSON.stringify({
              type:      'ice',
              agent:     agentId,
              candidate: e.candidate.toJSON(),  // includes usernameFragment (required by Firefox)
            }))
          }

          // Receive incoming video track from GStreamer
          pc.ontrack = (e) => {
            const video = videoRef.current
            if (!video || !e.streams[0]) return
            video.srcObject = e.streams[0]
            setConnected(true)

            // FPS counter using requestVideoFrameCallback
            // Generation counter prevents stacked tick chains if ontrack fires more than once
            if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
              const gen = ++fpsGenRef.current
              const tick = () => {
                if (closed || fpsGenRef.current !== gen) return
                frameCountRef.current++
                const now     = performance.now()
                const elapsed = now - fpsWindowRef.current
                if (elapsed >= 2000) {
                  setFps(Math.round((frameCountRef.current / elapsed) * 1000))
                  frameCountRef.current = 0
                  fpsWindowRef.current  = now
                }
                video.requestVideoFrameCallback(tick)
              }
              video.requestVideoFrameCallback(tick)
            }
          }

          pc.oniceconnectionstatechange = () => {
            if (
              pc.iceConnectionState === 'disconnected' ||
              pc.iceConnectionState === 'failed'
            ) {
              setConnected(false)
            }
          }

          // Apply offer → create answer → send immediately (trickle ICE)
          await pc.setRemoteDescription({ type: 'offer', sdp: msg.sdp })
          const answer = await pc.createAnswer()
          await pc.setLocalDescription(answer)

          // Send answer immediately — candidates trickle in via onicecandidate
          ws.send(JSON.stringify({
            type:  'answer',
            agent: agentId,
            sdp:   pc.localDescription.sdp,
          }))
        }

        // ── Incoming ICE candidate (from GStreamer producer) ──────────────────
        else if (msg.type === 'ice' && pcRef.current) {
          try {
            await pcRef.current.addIceCandidate(msg.candidate)
          } catch (e) {
            // Benign: can arrive before setRemoteDescription in rare timing
            console.warn('[useWebRTC] addIceCandidate failed:', e)
          }
        }
      }

      ws.onclose = () => {
        setConnected(false)
        if (!closed) setTimeout(connect, 3000)
      }

      ws.onerror = () => ws.close()
    }

    connect()

    return () => {
      closed = true
      closePc()
      wsRef.current?.close()
      wsRef.current = null
    }
  }, [agentId]) // eslint-disable-line react-hooks/exhaustive-deps

  return { videoRef, connected, fps }
}
