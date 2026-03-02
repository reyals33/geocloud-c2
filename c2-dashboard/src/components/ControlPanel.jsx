import { useEffect, useRef, useCallback } from 'react'
import Joystick from './Joystick'
import { useCmdVel } from '../hooks/useAgent'
import { AGENT_MAP, LINEAR_SPEED, ANGULAR_SPEED, DRONE_Z_SPEED } from '../config'

const PUBLISH_MS = 100  // 10 Hz publish rate

export default function ControlPanel({ agentId }) {
  const agent   = AGENT_MAP[agentId]
  const isDrone = agent.type === 'drone'
  const publish = useCmdVel(agentId)

  // Current joystick values — refs so the interval always reads latest without recreating
  const mainRef = useRef({ x: 0, y: 0 })  // x = yaw/lateral, y = forward
  const altRef  = useRef({ x: 0, y: 0 })  // y = altitude (drones only)

  // Continuous publish loop — sends zeros when sticks are centered (fixes VelocityControl latch)
  useEffect(() => {
    const id = setInterval(() => {
      const { x: jx, y: jy } = mainRef.current
      const { y: jz }        = altRef.current
      publish(
        { x: jy * LINEAR_SPEED,  z: isDrone ? jz * DRONE_Z_SPEED : 0 },
        { z: -jx * ANGULAR_SPEED }
      )
    }, PUBLISH_MS)
    return () => clearInterval(id)
  }, [publish, isDrone])

  const onMain = useCallback((v) => { mainRef.current = v }, [])
  const onAlt  = useCallback((v) => { altRef.current  = v }, [])

  const hardStop = useCallback(() => {
    mainRef.current = { x: 0, y: 0 }
    altRef.current  = { x: 0, y: 0 }
    publish({ x: 0, z: 0 }, { z: 0 })
  }, [publish])

  return (
    <div className="card">
      <div className="card-title">Command — {isDrone ? 'Drone' : 'Ground Robot'}</div>

      <div className="ctrl-sticks">
        <Joystick onChange={onMain} label={isDrone ? 'Fwd / Yaw' : 'Drive'} />
        {isDrone && (
          <Joystick onChange={onAlt} axisX={false} label="Alt" />
        )}
      </div>

      <button
        className="ctrl-stop-btn"
        onPointerDown={hardStop}
      >
        ■ STOP
      </button>
    </div>
  )
}
