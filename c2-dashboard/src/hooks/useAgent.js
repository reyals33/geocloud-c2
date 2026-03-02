import { useCallback, useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { useRos } from '../ros/RosContext'
import { quatToYaw } from '../utils/three'

/**
 * Subscribes to /{agentId}/odom and returns live telemetry.
 *
 * RTT is measured as a true WebSocket round-trip via /rosapi/get_time:
 *   browser → rosbridge (EC2) → service response → browser
 * This gives honest network latency, independent of Gazebo sim-clock.
 * (Gazebo stamps messages with simulation time starting at 0, not wall time.)
 */
export function useAgent(agentId) {
  const { ros, status } = useRos()
  const subRef = useRef(null)

  const [pose,      setPose]      = useState(null)
  const [rtt,       setRtt]       = useState(null)   // ms, WebSocket round-trip
  const [connected, setConnected] = useState(false)

  // ── Odom subscription ────────────────────────────────────────────────────────
  useEffect(() => {
    if (!agentId || status !== 'connected' || !ros.current) return

    const sub = new ROSLIB.Topic({
      ros:           ros.current,
      name:          `/${agentId}/odom`,
      messageType:   'nav_msgs/Odometry',
      throttle_rate: 50,
    })

    sub.subscribe(msg => {
      setConnected(true)
      const p = msg.pose.pose.position
      const o = msg.pose.pose.orientation
      setPose({ x: p.x, y: p.y, z: p.z, yaw: quatToYaw(o) })
    })

    subRef.current = sub
    return () => {
      sub.unsubscribe()
      subRef.current = null
      setConnected(false)
    }
  }, [agentId, status, ros])

  // ── Network RTT via /rosapi/get_time ping (every 2 s) ────────────────────────
  // /rosapi/get_time is a tiny service call that roundtrips through rosbridge.
  // It gives true browser↔rosbridge network latency, regardless of sim clock.
  useEffect(() => {
    if (status !== 'connected' || !ros.current) return

    const svc = new ROSLIB.Service({
      ros:         ros.current,
      name:        '/rosapi/get_time',
      serviceType: 'rosapi/GetTime',
    })

    function ping() {
      const t0 = performance.now()
      svc.callService(
        new ROSLIB.ServiceRequest({}),
        () => { setRtt(Math.round(performance.now() - t0)) },
        ()  => { setRtt(null) },
      )
    }

    ping()
    const id = setInterval(ping, 2000)
    return () => clearInterval(id)
  }, [status, ros])

  return { pose, rtt, connected }
}

/**
 * Returns a stable publish function for /{agentId}/cmd_vel.
 */
export function useCmdVel(agentId) {
  const { ros, status } = useRos()
  const pubRef = useRef(null)

  useEffect(() => {
    if (status !== 'connected' || !agentId) return

    pubRef.current = new ROSLIB.Topic({
      ros:         ros.current,
      name:        `/${agentId}/cmd_vel`,
      messageType: 'geometry_msgs/Twist',
    })

    return () => { pubRef.current = null }
  }, [agentId, status, ros])

  // Stable reference — pubRef.current is always fresh even though publish never changes
  const publish = useCallback((linear, angular) => {
    if (!pubRef.current) return
    pubRef.current.publish(new ROSLIB.Message({
      linear:  { x: linear.x  ?? 0, y: linear.y  ?? 0, z: linear.z  ?? 0 },
      angular: { x: 0,              y: 0,               z: angular.z ?? 0 },
    }))
  }, [])

  return publish
}
