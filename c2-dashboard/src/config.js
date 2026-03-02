// Agent definitions — single source of truth for IDs, labels, colors, types
// spawn: ROS-frame world origin for each agent (matches c2_arena.sdf spawn poses)
// Odom is relative to this frame, so overview scenes add it back to show world position.
export const AGENTS = [
  { id: 'robot1', label: 'Robot 1', color: '#0066ff', colorHex: 0x0066ff, type: 'ground', spawn: { x: -6, y: -6, z: 0   } },
  { id: 'robot2', label: 'Robot 2', color: '#00ff88', colorHex: 0x00ff88, type: 'ground', spawn: { x:  6, y:  6, z: 0   } },
  { id: 'drone1', label: 'Drone 1', color: '#ff3300', colorHex: 0xff3300, type: 'drone',  spawn: { x: -6, y:  6, z: 3.0 } },
  { id: 'drone2', label: 'Drone 2', color: '#ff8800', colorHex: 0xff8800, type: 'drone',  spawn: { x:  6, y: -6, z: 4.5 }, enhancedVision: true },
]

export const AGENT_MAP = Object.fromEntries(AGENTS.map(a => [a.id, a]))

// Direct rosbridge WebSocket on port 9090 (bypasses nginx proxy)
export const ROS_WS_URL = `ws://${window.location.hostname}:9090`

// WebRTC signaling server on port 8765 (direct — NOT through nginx proxy)
export const WEBRTC_URL = `ws://${window.location.hostname}:8765`

// cmd_vel speeds
export const LINEAR_SPEED  = 0.5   // m/s
export const ANGULAR_SPEED = 0.5   // rad/s
export const DRONE_Z_SPEED = 0.3   // m/s vertical
