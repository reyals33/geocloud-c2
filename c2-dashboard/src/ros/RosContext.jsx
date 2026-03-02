import { createContext, useContext, useEffect, useRef, useState } from 'react'
import ROSLIB from 'roslib'
import { ROS_WS_URL } from '../config'

const RosContext = createContext(null)

export function RosProvider({ children }) {
  const rosRef = useRef(null)
  const [status, setStatus] = useState('connecting') // 'connecting' | 'connected' | 'error'

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: ROS_WS_URL })
    rosRef.current = ros

    ros.on('connection', () => setStatus('connected'))
    ros.on('error',      () => setStatus('error'))
    ros.on('close',      () => {
      setStatus('connecting')
      // roslib auto-reconnects via its internal socket — no manual retry needed
    })

    return () => ros.close()
  }, [])

  return (
    <RosContext.Provider value={{ ros: rosRef, status }}>
      {children}
    </RosContext.Provider>
  )
}

export function useRos() {
  return useContext(RosContext)
}
