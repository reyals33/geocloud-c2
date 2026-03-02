/**
 * MiniMap — situational-awareness mini-map
 *
 * 180×180 orthographic top-down Three.js canvas showing all agents.
 * Current agent is highlighted with a pulsing ring.
 * Camera auto-zooms to keep all agents in frame.
 * Runs at ~30fps to stay lightweight as a secondary display.
 * Reuses the existing rosbridge WebSocket via RosContext.
 */

import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import ROSLIB from 'roslib'
import { useRos }     from '../ros/RosContext'
import { AGENTS, AGENT_MAP } from '../config'
import { r2t, quatToYaw, shiftTrail, disposeScene } from '../utils/three'

// ── Constants ─────────────────────────────────────────────────────────────────
const SIZE      = 180   // px — canvas is square
const TRAIL_LEN = 20    // shorter trail for mini-map
const LERP_K    = 0.10
const FPS_CAP   = 30    // render at 30fps max
const FRAME_MS  = 1000 / FPS_CAP

// ── Component ─────────────────────────────────────────────────────────────────
export default function MiniMap({ currentAgent }) {
  const mountRef   = useRef(null)
  const sceneRef   = useRef({})
  const targetsRef = useRef({})
  const curRef     = useRef({})
  const spanRef    = useRef(8)       // current camera half-span (lerped)
  const frameRef   = useRef(0)
  const lastRender = useRef(0)
  const clock      = useRef(new THREE.Clock())

  const { ros, status } = useRos()
  const agent = AGENT_MAP[currentAgent]

  // ── Three.js setup ──────────────────────────────────────────────────────────
  useEffect(() => {
    const mount = mountRef.current
    if (!mount) return

    // Renderer — fixed 180×180
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
    renderer.setSize(SIZE, SIZE)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.setClearColor(0x060810, 1)
    mount.appendChild(renderer.domElement)

    // Scene
    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x060810)

    // Soft ambient only — no directional shadows needed at this scale
    scene.add(new THREE.AmbientLight(0xffffff, 1.5))

    // Subtle grid
    const grid = new THREE.GridHelper(60, 30, 0x0d0d25, 0x0d0d25)
    grid.position.y = 0.01
    scene.add(grid)

    // Origin dot
    const origin = new THREE.Mesh(
      new THREE.CircleGeometry(0.18, 12),
      new THREE.MeshBasicMaterial({ color: 0x223366 })
    )
    origin.rotation.x = -Math.PI / 2
    origin.position.y = 0.02
    scene.add(origin)

    // ── Arena geometry (mirrors c2_arena.sdf, same coord system as r2t) ──────
    const mmWallMat = new THREE.MeshBasicMaterial({ color: 0x1a2744, transparent: true, opacity: 0.55 })
    ;[
      { pos: [0, 0, -10], size: [20.6, 0.3] },  // north wall (top-down: w×d)
      { pos: [0, 0,  10], size: [20.6, 0.3] },  // south
      { pos: [10, 0,  0], size: [0.3, 20.0] },  // east
      { pos: [-10, 0, 0], size: [0.3, 20.0] },  // west
    ].forEach(({ pos, size }) => {
      const m = new THREE.Mesh(
        new THREE.PlaneGeometry(size[0], size[1]),
        mmWallMat
      )
      m.rotation.x = -Math.PI / 2
      m.position.set(pos[0], 0.03, pos[2])
      scene.add(m)
    })

    const mmPillarMat = new THREE.MeshBasicMaterial({ color: 0x223355, transparent: true, opacity: 0.6 })
    ;[[5,5],[-5,5],[5,-5],[-5,-5],[0,7],[0,-7]].forEach(([px, py]) => {
      const m = new THREE.Mesh(new THREE.CircleGeometry(0.4, 12), mmPillarMat)
      m.rotation.x = -Math.PI / 2
      m.position.set(px, 0.03, -py)
      scene.add(m)
    })

    const mmObsMat = new THREE.MeshBasicMaterial({ color: 0x3a2000, transparent: true, opacity: 0.55 })
    ;[[3.5, 0], [-3.5, 0]].forEach(([ox, oy]) => {
      const m = new THREE.Mesh(new THREE.PlaneGeometry(1.2, 1.2), mmObsMat)
      m.rotation.x = -Math.PI / 2
      m.position.set(ox, 0.03, -oy)
      scene.add(m)
    })

    // Orthographic camera — perfect top-down, no tilt (cleaner at small size)
    const vs = spanRef.current
    const camera = new THREE.OrthographicCamera(-vs, vs, vs, -vs, 0.1, 200)
    camera.position.set(0, 30, 0)
    camera.up.set(0, 0, -1)
    camera.lookAt(0, 0, 0)

    // ── Agents ────────────────────────────────────────────────────────────────
    const meshes = {}
    const trails = {}
    let pulseRing = null, pulseRingMat = null

    AGENTS.forEach(agent => {
      const col     = new THREE.Color(agent.color)
      const isDrone = agent.type === 'drone'
      const isSelf  = agent.id === currentAgent

      // Mesh — current agent is larger
      const size   = isSelf ? 0.7 : 0.45
      const geo    = isDrone
        ? new THREE.SphereGeometry(size * 0.9, 12, 8)
        : new THREE.BoxGeometry(size, size * 0.4, size * 1.2)
      const mat    = new THREE.MeshStandardMaterial({
        color:             col,
        emissive:          col,
        emissiveIntensity: isSelf ? 1.4 : 0.6,
        roughness:         0.3, metalness: 0.6,
      })
      const mesh   = new THREE.Mesh(geo, mat)
      mesh.position.y = isDrone ? 0.5 : 0.2
      scene.add(mesh)
      meshes[agent.id] = mesh

      // Pulsing ring around current agent
      if (isSelf) {
        pulseRingMat = new THREE.MeshBasicMaterial({
          color: agent.colorHex, transparent: true, opacity: 0.7, side: THREE.DoubleSide,
        })
        pulseRing = new THREE.Mesh(new THREE.RingGeometry(0.85, 1.1, 36), pulseRingMat)
        pulseRing.rotation.x = -Math.PI / 2
        pulseRing.position.y = 0.05
        mesh.add(pulseRing)
      }

      // Trail
      const posArr  = new Float32Array(TRAIL_LEN * 3)
      const colArr  = new Float32Array(TRAIL_LEN * 3)
      const pts     = Array.from({ length: TRAIL_LEN }, () => new THREE.Vector3())
      const trailGeo = new THREE.BufferGeometry()
      trailGeo.setAttribute('position', new THREE.BufferAttribute(posArr, 3))
      trailGeo.setAttribute('color',    new THREE.BufferAttribute(colArr, 3))
      trailGeo.setDrawRange(0, 0)
      const trailLine = new THREE.Line(
        trailGeo,
        new THREE.LineBasicMaterial({
          vertexColors: true, transparent: true,
          opacity: isSelf ? 0.9 : 0.5,
        })
      )
      trailLine.frustumCulled = false
      scene.add(trailLine)
      trails[agent.id] = { line: trailLine, pts, posArr, colArr, count: 0, color: col }
    })

    // ── Animation loop ────────────────────────────────────────────────────────
    let animId

    function animate() {
      animId = requestAnimationFrame(animate)
      const now = performance.now()
      frameRef.current++

      // Lerp agents
      AGENTS.forEach(ag => {
        const target = targetsRef.current[ag.id]
        const mesh   = meshes[ag.id]
        const trail  = trails[ag.id]
        if (!target || !mesh) return

        if (!curRef.current[ag.id])
          curRef.current[ag.id] = { x: target.x, y: target.y, z: target.z }
        const cur = curRef.current[ag.id]

        cur.x = THREE.MathUtils.lerp(cur.x, target.x, LERP_K)
        cur.y = THREE.MathUtils.lerp(cur.y, target.y, LERP_K)
        cur.z = THREE.MathUtils.lerp(cur.z, target.z, LERP_K)

        const isDrone = ag.type === 'drone'
        const wp      = r2t(cur.x, cur.y, cur.z)
        mesh.position.x = wp.x
        mesh.position.y = isDrone ? Math.max(0.5, wp.y * 0.4) : 0.2
        mesh.position.z = wp.z

        if (!isDrone) mesh.rotation.y = -(target.yaw ?? 0)

        if (frameRef.current % 2 === 0)
          shiftTrail(trail, wp.x, isDrone ? 0.05 : 0.03, wp.z)
      })

      // Pulse ring animation on current agent
      if (pulseRing && pulseRingMat) {
        const t     = clock.current.getElapsedTime()
        const pulse = 1 + 0.22 * Math.sin(t * 2.8)
        pulseRing.scale.set(pulse, pulse, pulse)
        pulseRingMat.opacity = 0.5 + 0.3 * Math.sin(t * 2.8)
      }

      // Auto-zoom: expand camera to fit all agents with padding
      const positions = Object.values(curRef.current)
      if (positions.length > 0) {
        let minX = -4, maxX = 4, minZ = -4, maxZ = 4
        positions.forEach(({ x, y }) => {
          const wx = x, wz = -y
          minX = Math.min(minX, wx - 2); maxX = Math.max(maxX, wx + 2)
          minZ = Math.min(minZ, wz - 2); maxZ = Math.max(maxZ, wz + 2)
        })
        const spanX = (maxX - minX) / 2
        const spanZ = (maxZ - minZ) / 2
        const target = Math.max(spanX, spanZ) * 1.25

        // Lerp the camera span
        spanRef.current = THREE.MathUtils.lerp(spanRef.current, target, 0.015)
        const s = spanRef.current
        camera.left = -s; camera.right  = s
        camera.top  =  s; camera.bottom = -s
        camera.updateProjectionMatrix()

        // Re-center camera on midpoint of all agents
        const cx = (minX + maxX) / 2
        const cz = (minZ + maxZ) / 2
        camera.position.x = THREE.MathUtils.lerp(camera.position.x, cx, 0.015)
        camera.position.z = THREE.MathUtils.lerp(camera.position.z, cz, 0.015)
        camera.lookAt(camera.position.x, 0, camera.position.z)
      }

      // 30fps render cap — skip render on high-frequency frames
      if (now - lastRender.current >= FRAME_MS) {
        renderer.render(scene, camera)
        lastRender.current = now
      }
    }
    animate()

    sceneRef.current = { renderer, scene, camera, meshes, trails }

    return () => {
      cancelAnimationFrame(animId)
      disposeScene(scene)
      if (mount.contains(renderer.domElement)) mount.removeChild(renderer.domElement)
      renderer.dispose()
    }
  }, [currentAgent]) // eslint-disable-line react-hooks/exhaustive-deps

  // ── ROS subscriptions (all agents) ────────────────────────────────────────
  useEffect(() => {
    if (status !== 'connected' || !ros.current) return
    const subs = []

    AGENTS.forEach(ag => {
      const sub = new ROSLIB.Topic({
        ros:         ros.current,
        name:        `/${ag.id}/odom`,
        messageType: 'nav_msgs/Odometry',
        throttle_rate: 100,  // 10Hz enough for mini-map
      })
      sub.subscribe(msg => {
        const p = msg.pose.pose.position
        const s = ag.spawn
        targetsRef.current[ag.id] = {
          x: p.x + s.x, y: p.y + s.y, z: p.z + s.z,
          yaw: quatToYaw(msg.pose.pose.orientation),
        }
      })
      subs.push(sub)
    })

    return () => subs.forEach(s => s.unsubscribe())
  }, [status, ros])

  return (
    <div className="minimap-wrap" style={{ '--mm-color': agent.color }}>
      <div className="minimap-label" style={{ color: agent.color }}>
        TACTICAL OVERVIEW
      </div>
      <div ref={mountRef} style={{ width: SIZE, height: SIZE }} />
    </div>
  )
}
