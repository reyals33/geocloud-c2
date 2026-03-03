/**
 * OverviewScene v3 — Mission Control × Orbital Robotics × Precision Telemetry
 *
 * Three.js showcase elements:
 *   · Custom GLSL ground plane — anti-aliased 1-unit + 5-unit grid via fwidth() derivatives
 *     + animated concentric pulse rings expanding from mission origin
 *   · Drone: translucent shell + inner glowing core + spinning orbital ring + altitude leader line
 *   · Ground robot: precision flat chassis + hot sensor pod
 *   · Ground-projection rings below every agent
 *   · 200-point vertex-colour trails (squared-alpha fade for comet-tail effect)
 *   · L-bracket ground reticles (slowly rotating)
 *   · UnrealBloom (0.65, threshold 0.45 — selective to emissives only)
 *   · ACESFilmic tone mapping
 *   · Isometric OrthographicCamera — auto-orbit ON by default
 *   · NASA-style HUD: mission elapsed time · agent roster · ROS status
 */

import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { EffectComposer }  from 'three/examples/jsm/postprocessing/EffectComposer.js'
import { RenderPass }      from 'three/examples/jsm/postprocessing/RenderPass.js'
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js'
import { CSS2DRenderer, CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer.js'
import ROSLIB from 'roslib'
import { useRos }  from '../ros/RosContext'
import { AGENTS }  from '../config'
import { r2t, quatToYaw, shiftTrail, disposeScene } from '../utils/three'

// ── Constants ─────────────────────────────────────────────────────────────────
const TRAIL_LEN = 200
const LERP_K    = 0.10
const ISO_R     = 24
const ISO_Y     = 22
const ORBIT_SPD = 0.012   // rad/s — slow cinematic revolution

// ── Ground plane shaders ──────────────────────────────────────────────────────
const GROUND_VERT = `
varying vec2 vWorldXZ;
void main() {
  vWorldXZ    = position.xz;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
`

const GROUND_FRAG = `
#extension GL_OES_standard_derivatives : enable
precision highp float;

uniform float uTime;
varying vec2  vWorldXZ;

void main() {
  vec2  p    = vWorldXZ;
  float dist = length(p);

  // Anti-aliased fine grid (1-unit spacing)
  vec2  gf = abs(fract(p - 0.5) - 0.5) / fwidth(p);
  float lf = 1.0 - min(min(gf.x, gf.y), 1.0);

  // Anti-aliased coarse grid (5-unit spacing)
  vec2  gc = abs(fract(p / 5.0 - 0.5) - 0.5) / (fwidth(p) * 5.0);
  float lc = 1.0 - min(min(gc.x, gc.y), 1.0);

  // Origin axis lines (X and Z)
  vec2  axPx = abs(p) / fwidth(p);
  float la   = 1.0 - min(min(axPx.x, axPx.y), 1.0);

  // Distance attenuation — grid fades at perimeter
  float fade = 1.0 - smoothstep(10.0, 34.0, dist);

  // Animated pulse ring expanding from origin (~5.5 s cycle)
  float phase = fract(uTime * 0.18) * 30.0;
  float ring  = exp(-pow(abs(dist - phase), 2.0) * 0.28)
              * (1.0 - smoothstep(0.0, 30.0, phase));

  vec3 fineCol   = vec3(0.00, 0.09, 0.34);
  vec3 coarseCol = vec3(0.00, 0.22, 0.70);
  vec3 axisCol   = vec3(0.00, 0.48, 1.00);
  vec3 ringCol   = vec3(0.00, 0.55, 1.00);

  vec3  col   = fineCol   * lf * 0.55 * fade
              + coarseCol * lc        * fade
              + axisCol   * la * 0.9
              + ringCol   * ring * 0.75;
  float alpha = (lf * 0.28 + lc * 0.62) * fade
              + la   * 0.55
              + ring * 0.55;

  gl_FragColor = vec4(col, clamp(alpha, 0.0, 0.95));
}
`

// ── Helpers ───────────────────────────────────────────────────────────────────
function triggerPulse(pool, x, z, colorHex) {
  const slot = pool.find(p => !p.active)
  if (!slot) return
  slot.active = true
  slot.t      = 0
  slot.ring.material.color.setHex(colorHex)
  slot.ring.material.opacity = 0.7
  slot.ring.position.set(x, 0.08, z)
  slot.ring.scale.set(1, 1, 1)
  slot.ring.visible = true
}

function createGroundReticle(colorHex) {
  const group = new THREE.Group()
  const mat   = new THREE.LineBasicMaterial({ color: colorHex, transparent: true, opacity: 0.72 })
  const sz = 0.88, gap = 0.36, y = 0.04
  ;[1, -1].forEach(xs => {
    ;[1, -1].forEach(zs => {
      const gX = new THREE.BufferGeometry()
      gX.setAttribute('position', new THREE.BufferAttribute(
        new Float32Array([xs*gap, y, zs*sz, xs*sz, y, zs*sz]), 3))
      group.add(new THREE.Line(gX, mat))
      const gZ = new THREE.BufferGeometry()
      gZ.setAttribute('position', new THREE.BufferAttribute(
        new Float32Array([xs*sz, y, zs*gap, xs*sz, y, zs*sz]), 3))
      group.add(new THREE.Line(gZ, mat))
    })
  })
  return group
}

// ── Component ─────────────────────────────────────────────────────────────────
export default function OverviewScene() {
  const mountRef   = useRef(null)
  const sceneRef   = useRef({})
  const targetsRef = useRef({})
  const curRef     = useRef({})
  const orbitRef   = useRef(true)    // orbit ON by default for maximum visual impact
  const panRef     = useRef({ x: 0, z: 0 })
  const viewSzRef  = useRef(16)
  const clock      = useRef(new THREE.Clock())
  const frameRef   = useRef(0)

  const { ros, status } = useRos()
  const [orbitMode,   setOrbitMode]   = useState(true)
  const [hudData,     setHudData]     = useState({})
  const [agentCoords, setAgentCoords] = useState({})
  const [missionSec,  setMissionSec]  = useState(0)

  // ── Mission elapsed time ───────────────────────────────────────────────────
  useEffect(() => {
    const start = Date.now()
    const id = setInterval(() => setMissionSec(Math.floor((Date.now() - start) / 1000)), 1000)
    return () => clearInterval(id)
  }, [])

  // ── Three.js setup ────────────────────────────────────────────────────────
  useEffect(() => {
    const mount  = mountRef.current
    const W      = mount.clientWidth
    const H      = mount.clientHeight
    const aspect = W / H

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' })
    renderer.setSize(W, H)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    renderer.toneMapping         = THREE.ACESFilmicToneMapping
    renderer.toneMappingExposure = 1.35
    mount.appendChild(renderer.domElement)

    // CSS2D label renderer
    const labelRenderer = new CSS2DRenderer()
    labelRenderer.setSize(W, H)
    Object.assign(labelRenderer.domElement.style, {
      position: 'absolute', top: '0', pointerEvents: 'none',
    })
    mount.appendChild(labelRenderer.domElement)

    // Scene
    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x000005)
    scene.fog = new THREE.FogExp2(0x000005, 0.0022)
    scene.add(new THREE.HemisphereLight(0x001833, 0x000000, 1.2))

    // ── Starfield ─────────────────────────────────────────────────────────────
    // Dim background fill stars
    const s1Pos = new Float32Array(700 * 3)
    for (let i = 0; i < 700; i++) {
      const th = Math.random() * Math.PI * 2
      const ph = Math.acos(2 * Math.random() - 1)
      const r  = 100 + Math.random() * 60
      s1Pos[i*3]   = r * Math.sin(ph) * Math.cos(th)
      s1Pos[i*3+1] = Math.abs(r * Math.sin(ph) * Math.sin(th))
      s1Pos[i*3+2] = r * Math.cos(ph)
    }
    const s1G = new THREE.BufferGeometry()
    s1G.setAttribute('position', new THREE.BufferAttribute(s1Pos, 3))
    scene.add(new THREE.Points(s1G, new THREE.PointsMaterial({
      color: 0x223355, size: 0.08, sizeAttenuation: true, transparent: true, opacity: 0.40,
    })))

    // Bright accent stars
    const s2Pos = new Float32Array(100 * 3)
    for (let i = 0; i < 100; i++) {
      const th = Math.random() * Math.PI * 2
      const ph = Math.acos(2 * Math.random() - 1)
      const r  = 85 + Math.random() * 35
      s2Pos[i*3]   = r * Math.sin(ph) * Math.cos(th)
      s2Pos[i*3+1] = Math.abs(r * Math.sin(ph) * Math.sin(th))
      s2Pos[i*3+2] = r * Math.cos(ph)
    }
    const s2G = new THREE.BufferGeometry()
    s2G.setAttribute('position', new THREE.BufferAttribute(s2Pos, 3))
    scene.add(new THREE.Points(s2G, new THREE.PointsMaterial({
      color: 0x99ccff, size: 0.20, sizeAttenuation: true, transparent: true, opacity: 0.55,
    })))

    // ── Ground plane — custom GLSL shader ────────────────────────────────────
    const groundUniforms = { uTime: { value: 0 } }
    const groundMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(80, 80),
      new THREE.ShaderMaterial({
        extensions:     { derivatives: true },
        uniforms:       groundUniforms,
        vertexShader:   GROUND_VERT,
        fragmentShader: GROUND_FRAG,
        transparent:    true,
        side:           THREE.DoubleSide,
        depthWrite:     false,
      })
    )
    groundMesh.rotation.x = -Math.PI / 2
    groundMesh.position.y = 0.002
    scene.add(groundMesh)

    // ── Range reference circles ───────────────────────────────────────────────
    ;[5, 10, 15].forEach((rad, i) => {
      const geo = new THREE.RingGeometry(rad - 0.016, rad + 0.016, 128)
      geo.rotateX(-Math.PI / 2)
      const mesh = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
        color: [0x001a44, 0x001e55, 0x002266][i],
        transparent: true, opacity: 0.65, side: THREE.DoubleSide, depthWrite: false,
      }))
      mesh.position.y = 0.006
      scene.add(mesh)
    })

    // ── Arena walls ───────────────────────────────────────────────────────────
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x06101e, emissive: 0x050c20, emissiveIntensity: 0.45,
      transparent: true, opacity: 0.72, roughness: 0.9,
    })
    ;[
      { pos: [0, 0.1, -10], size: [20.6, 2, 0.28] },
      { pos: [0, 0.1,  10], size: [20.6, 2, 0.28] },
      { pos: [ 10, 0.1, 0], size: [0.28, 2, 20]   },
      { pos: [-10, 0.1, 0], size: [0.28, 2, 20]   },
    ].forEach(({ pos, size }) => {
      const m = new THREE.Mesh(new THREE.BoxGeometry(...size), wallMat)
      m.position.set(...pos)
      scene.add(m)
    })

    const edgeMat = new THREE.LineBasicMaterial({ color: 0x001533, transparent: true, opacity: 0.28 })
    ;[
      [[-10.3,2,-10],[10.3,2,-10]], [[-10.3,2,10],[10.3,2,10]],
      [[10,2,-10],[10,2,10]],       [[-10,2,-10],[-10,2,10]],
    ].forEach(([a, b]) => {
      const geo = new THREE.BufferGeometry()
      geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array([...a, ...b]), 3))
      scene.add(new THREE.Line(geo, edgeMat))
    })

    // Pillars
    const pillarMat = new THREE.MeshStandardMaterial({ color: 0x08121f, emissive: 0x060a18, emissiveIntensity: 0.4, roughness: 0.75 })
    const crownMat  = new THREE.MeshBasicMaterial({ color: 0x001233, transparent: true, opacity: 0.32 })
    ;[[5,5],[-5,5],[5,-5],[-5,-5],[0,7],[0,-7]].forEach(([px,py]) => {
      const p = new THREE.Mesh(new THREE.CylinderGeometry(0.33, 0.33, 2.5, 16), pillarMat)
      p.position.set(px, 1.25, -py)
      scene.add(p)
      const c = new THREE.Mesh(new THREE.TorusGeometry(0.38, 0.035, 6, 24), crownMat)
      c.rotation.x = Math.PI / 2
      c.position.set(px, 2.52, -py)
      scene.add(c)
    })

    // Obstacles
    const obsMat = new THREE.MeshStandardMaterial({ color: 0x160800, emissive: 0x0c0400, emissiveIntensity: 0.3, roughness: 0.95 })
    ;[[3.5,0],[-3.5,0]].forEach(([ox,oy]) => {
      const m = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1, 1.2), obsMat)
      m.position.set(ox, 0.5, -oy)
      scene.add(m)
    })

    // ── Camera ────────────────────────────────────────────────────────────────
    let vs = viewSzRef.current
    const camera = new THREE.OrthographicCamera(-vs*aspect, vs*aspect, vs, -vs, 0.1, 400)
    camera.position.set(ISO_R, ISO_Y, ISO_R)
    camera.up.set(0, 1, 0)
    camera.lookAt(0, 0, 0)

    // ── Bloom ─────────────────────────────────────────────────────────────────
    let composer = null, useBloom = false
    try {
      composer = new EffectComposer(renderer)
      composer.addPass(new RenderPass(scene, camera))
      composer.addPass(new UnrealBloomPass(new THREE.Vector2(W, H), 0.65, 0.40, 0.45))
      useBloom = true
    } catch(e) { console.warn('Bloom unavailable:', e) }

    // ── Agents ────────────────────────────────────────────────────────────────
    const meshes = {}, trails = {}, reticles = {}, altLines = {}, groundCircles = {}, droneOrbits = {}

    AGENTS.forEach(agent => {
      const col     = new THREE.Color(agent.color)
      const isDrone = agent.type === 'drone'

      // ── Agent mesh ──────────────────────────────────────────────────────────
      let mesh
      if (isDrone) {
        // Translucent shell + inner emissive core
        const shell = new THREE.Mesh(
          new THREE.SphereGeometry(0.42, 32, 24),
          new THREE.MeshStandardMaterial({
            color: col, emissive: col, emissiveIntensity: 0.45,
            transparent: true, opacity: 0.22, roughness: 0.05, metalness: 0.9,
          })
        )
        const core = new THREE.Mesh(
          new THREE.SphereGeometry(0.19, 16, 12),
          new THREE.MeshStandardMaterial({
            color: col, emissive: col, emissiveIntensity: 3.2,
            roughness: 0.03, metalness: 0.95,
          })
        )
        mesh = new THREE.Group()
        mesh.add(shell)
        mesh.add(core)
        mesh.position.y = 2.5

        // Spinning orbital ring
        const orbit = new THREE.Mesh(
          new THREE.TorusGeometry(0.66, 0.020, 8, 64),
          new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.68 })
        )
        orbit.rotation.x = Math.PI * 0.28
        mesh.add(orbit)
        droneOrbits[agent.id] = orbit

      } else {
        // Precision flat chassis + hot sensor pod
        const chassis = new THREE.Mesh(
          new THREE.BoxGeometry(0.80, 0.16, 1.10),
          new THREE.MeshStandardMaterial({
            color: col, emissive: col, emissiveIntensity: 1.8, roughness: 0.18, metalness: 0.88,
          })
        )
        const pod = new THREE.Mesh(
          new THREE.BoxGeometry(0.26, 0.09, 0.30),
          new THREE.MeshStandardMaterial({
            color: col, emissive: col, emissiveIntensity: 3.8, roughness: 0.04,
          })
        )
        pod.position.y = 0.125
        mesh = new THREE.Group()
        mesh.add(chassis)
        mesh.add(pod)
        mesh.position.y = 0.08
      }
      scene.add(mesh)
      meshes[agent.id] = mesh

      // Point light
      const light = new THREE.PointLight(agent.colorHex, 6, 14)
      light.position.y = 0
      mesh.add(light)

      // Altitude leader line (drones — shows height above ground)
      if (isDrone) {
        const altGeo = new THREE.BufferGeometry()
        altGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3))
        const altLine = new THREE.Line(altGeo, new THREE.LineBasicMaterial({
          color: agent.colorHex, transparent: true, opacity: 0.25,
        }))
        altLine.frustumCulled = false
        scene.add(altLine)
        altLines[agent.id] = altLine
      }

      // Ground projection ring
      const ringR = isDrone ? [0.22, 0.34] : [0.36, 0.50]
      const circleGeo = new THREE.RingGeometry(ringR[0], ringR[1], 48)
      circleGeo.rotateX(-Math.PI / 2)
      const groundCircle = new THREE.Mesh(circleGeo, new THREE.MeshBasicMaterial({
        color: agent.colorHex, transparent: true, opacity: 0.18,
        side: THREE.DoubleSide, depthWrite: false,
      }))
      groundCircle.position.y = 0.018
      scene.add(groundCircle)
      groundCircles[agent.id] = groundCircle

      // Trail
      const posArr = new Float32Array(TRAIL_LEN * 3)
      const colArr = new Float32Array(TRAIL_LEN * 3)
      const pts    = Array.from({ length: TRAIL_LEN }, () => new THREE.Vector3())
      const tGeo   = new THREE.BufferGeometry()
      tGeo.setAttribute('position', new THREE.BufferAttribute(posArr, 3))
      tGeo.setAttribute('color',    new THREE.BufferAttribute(colArr, 3))
      tGeo.setDrawRange(0, 0)
      const tLine  = new THREE.Line(tGeo,
        new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.90 }))
      tLine.frustumCulled = false
      scene.add(tLine)
      trails[agent.id] = { line: tLine, pts, posArr, colArr, count: 0, color: col }

      // Ground reticle
      const reticle = createGroundReticle(agent.colorHex)
      scene.add(reticle)
      reticles[agent.id] = reticle

      // CSS2D label
      const div = document.createElement('div')
      div.className = 'c2-label'
      div.innerHTML = `
        <div class="c2-label-name" style="color:${agent.color}">
          ${agent.id.replace(/([a-z])(\d)/, '$1-$2').toUpperCase()}
        </div>
        <div class="c2-label-status" id="c2sta-${agent.id}">OFFLINE</div>
      `
      const labelObj = new CSS2DObject(div)
      labelObj.position.set(0, isDrone ? 1.4 : 0.85, 0)
      mesh.add(labelObj)
    })

    // ── Pulse ring pool ───────────────────────────────────────────────────────
    const pulsePool = []
    for (let i = 0; i < 8; i++) {
      const geo = new THREE.RingGeometry(0.04, 0.22, 48)
      geo.rotateX(-Math.PI / 2)
      const ring = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
        color: 0xffffff, transparent: true, opacity: 0, side: THREE.DoubleSide,
      }))
      ring.visible = false
      scene.add(ring)
      pulsePool.push({ ring, active: false, t: 0 })
    }

    // ── Mouse: pan + zoom ─────────────────────────────────────────────────────
    let dragging = false, lastMX = 0, lastMY = 0
    const onDown  = e => { dragging = true; lastMX = e.clientX; lastMY = e.clientY }
    const onMove  = e => {
      if (!dragging) return
      const speed = (viewSzRef.current * 2) / mount.clientHeight
      panRef.current.x -= (e.clientX - lastMX) * speed * 0.7
      panRef.current.z += (e.clientY - lastMY) * speed * 0.7
      lastMX = e.clientX; lastMY = e.clientY
    }
    const onUp    = () => { dragging = false }
    const onWheel = e => {
      e.preventDefault()
      viewSzRef.current = Math.max(8, Math.min(60, viewSzRef.current + e.deltaY * 0.05))
      const a = mount.clientWidth / mount.clientHeight, v = viewSzRef.current
      camera.left = -v*a; camera.right = v*a; camera.top = v; camera.bottom = -v
      camera.updateProjectionMatrix()
    }
    mount.addEventListener('mousedown',  onDown)
    mount.addEventListener('mousemove',  onMove)
    mount.addEventListener('mouseup',    onUp)
    mount.addEventListener('mouseleave', onUp)
    mount.addEventListener('wheel', onWheel, { passive: false })

    // ── Resize ────────────────────────────────────────────────────────────────
    const onResize = () => {
      const nw = mount.clientWidth, nh = mount.clientHeight, a = nw/nh, v = viewSzRef.current
      renderer.setSize(nw, nh)
      labelRenderer.setSize(nw, nh)
      if (composer) composer.setSize(nw, nh)
      camera.left = -v*a; camera.right = v*a; camera.top = v; camera.bottom = -v
      camera.updateProjectionMatrix()
    }
    window.addEventListener('resize', onResize)
    onResize()  // correct size immediately — clientHeight may be 0 at initial read

    // ── Animation loop ────────────────────────────────────────────────────────
    let animId
    function animate() {
      animId = requestAnimationFrame(animate)
      const dt = clock.current.getDelta()
      const et = clock.current.getElapsedTime()
      frameRef.current++

      // Update ground shader time
      groundUniforms.uTime.value = et

      // Camera orbit
      const pan = panRef.current
      if (orbitRef.current) {
        const angle = et * ORBIT_SPD
        camera.position.set(pan.x + ISO_R * Math.sin(angle), ISO_Y, pan.z + ISO_R * Math.cos(angle))
      } else {
        camera.position.set(pan.x + ISO_R, ISO_Y, pan.z + ISO_R)
      }
      camera.lookAt(pan.x, 0, pan.z)

      // Drone orbital rings
      Object.values(droneOrbits).forEach(orbit => { orbit.rotation.z += dt * 1.2 })

      // Agents
      AGENTS.forEach(agent => {
        const target  = targetsRef.current[agent.id]
        const mesh    = meshes[agent.id]
        const trail   = trails[agent.id]
        const reticle = reticles[agent.id]
        if (!target || !mesh || !trail) return

        if (!curRef.current[agent.id])
          curRef.current[agent.id] = { x: target.x, y: target.y, z: target.z }
        const cur = curRef.current[agent.id]

        cur.x = THREE.MathUtils.lerp(cur.x, target.x, LERP_K)
        cur.y = THREE.MathUtils.lerp(cur.y, target.y, LERP_K)
        cur.z = THREE.MathUtils.lerp(cur.z, target.z, LERP_K)

        const isDrone = agent.type === 'drone'
        const wp      = r2t(cur.x, cur.y, cur.z)
        if (isDrone) wp.y = Math.max(0.6, wp.y)
        else         wp.y = 0.08

        mesh.position.copy(wp)
        if (!isDrone) mesh.rotation.y = -(target.yaw ?? 0)

        // Trail (every other frame for performance)
        if (frameRef.current % 2 === 0) {
          shiftTrail(trail, wp.x, isDrone ? Math.max(0.08, wp.y * 0.4) : 0.05, wp.z)
        }

        // Altitude leader line for drones
        if (isDrone && altLines[agent.id]) {
          const pos = altLines[agent.id].geometry.attributes.position.array
          pos[0]=wp.x; pos[1]=0.02; pos[2]=wp.z
          pos[3]=wp.x; pos[4]=wp.y;  pos[5]=wp.z
          altLines[agent.id].geometry.attributes.position.needsUpdate = true
        }

        // Ground projection circle
        if (groundCircles[agent.id]) {
          groundCircles[agent.id].position.set(wp.x, 0.018, wp.z)
        }

        // Reticle follows agent, slowly rotates
        if (reticle) {
          reticle.position.set(wp.x, 0, wp.z)
          reticle.rotation.y += dt * 0.18
        }
      })

      // Pulse rings
      pulsePool.forEach(p => {
        if (!p.active) return
        p.t += dt
        if (p.t >= 1.2) { p.active = false; p.ring.visible = false; return }
        const prog = p.t / 1.2
        p.ring.scale.set(1 + prog * 9, 1, 1 + prog * 9)
        p.ring.material.opacity = (1 - prog) * 0.60
      })

      if (useBloom && composer) composer.render()
      else                      renderer.render(scene, camera)
      labelRenderer.render(scene, camera)
    }
    animate()

    sceneRef.current = { renderer, labelRenderer, composer, scene, camera, meshes, trails, pulsePool }

    return () => {
      cancelAnimationFrame(animId)
      window.removeEventListener('resize', onResize)
      mount.removeEventListener('mousedown',  onDown)
      mount.removeEventListener('mousemove',  onMove)
      mount.removeEventListener('mouseup',    onUp)
      mount.removeEventListener('mouseleave', onUp)
      mount.removeEventListener('wheel',      onWheel)
      disposeScene(scene)
      if (mount.contains(renderer.domElement))      mount.removeChild(renderer.domElement)
      if (mount.contains(labelRenderer.domElement)) mount.removeChild(labelRenderer.domElement)
      renderer.dispose()
      if (composer) composer.dispose()
    }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  // ── ROS subscriptions ─────────────────────────────────────────────────────
  useEffect(() => {
    if (status !== 'connected' || !ros.current) return
    const subs = []

    AGENTS.forEach(agent => {
      const odomSub = new ROSLIB.Topic({
        ros: ros.current, name: `/${agent.id}/odom`,
        messageType: 'nav_msgs/Odometry', throttle_rate: 50,
      })
      odomSub.subscribe(msg => {
        const p = msg.pose.pose.position
        const s = agent.spawn
        targetsRef.current[agent.id] = {
          x: p.x + s.x, y: p.y + s.y, z: p.z + s.z,
          yaw: quatToYaw(msg.pose.pose.orientation),
        }
        const el = document.getElementById(`c2sta-${agent.id}`)
        if (el) { el.textContent = 'ACTIVE'; el.className = 'c2-label-status active' }
        setHudData(prev => ({ ...prev, [agent.id]: { connected: true } }))
      })
      subs.push(odomSub)

      const velSub = new ROSLIB.Topic({
        ros: ros.current, name: `/${agent.id}/cmd_vel`,
        messageType: 'geometry_msgs/Twist', throttle_rate: 200,
      })
      velSub.subscribe(() => {
        const mesh = sceneRef.current.meshes?.[agent.id]
        if (mesh) triggerPulse(sceneRef.current.pulsePool, mesh.position.x, mesh.position.z, agent.colorHex)
      })
      subs.push(velSub)
    })

    return () => {
      subs.forEach(s => s.unsubscribe())
      setHudData({})
      AGENTS.forEach(a => {
        const el = document.getElementById(`c2sta-${a.id}`)
        if (el) { el.textContent = 'OFFLINE'; el.className = 'c2-label-status' }
      })
    }
  }, [status, ros])

  // ── Agent coords (5 Hz) ───────────────────────────────────────────────────
  useEffect(() => {
    const id = setInterval(() => {
      const coords = {}
      AGENTS.forEach(a => {
        const t = targetsRef.current[a.id]
        if (t) coords[a.id] = { x: t.x, y: t.y }
      })
      setAgentCoords(coords)
    }, 200)
    return () => clearInterval(id)
  }, [])

  const toggleOrbit = () => {
    orbitRef.current = !orbitRef.current
    setOrbitMode(orbitRef.current)
  }

  const fmtTime = s => {
    const h  = Math.floor(s / 3600)
    const m  = Math.floor((s % 3600) / 60)
    const sc = s % 60
    return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(sc).padStart(2,'0')}`
  }

  const onlineCount = AGENTS.filter(a => hudData[a.id]?.connected).length

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <div ref={mountRef} style={{ width: '100%', height: '100%' }} />

      <div className="c2-scanlines" />
      <div className="c2-vignette"  />

      <div className="c2-corner c2-corner-tl" />
      <div className="c2-corner c2-corner-tr" />
      <div className="c2-corner c2-corner-bl" />
      <div className="c2-corner c2-corner-br" />

      {/* MET — top-left */}
      <div className="c2-met">MET {fmtTime(missionSec)}</div>

      {/* Agent roster — top-right */}
      <div className="c2-roster-panel">
        {AGENTS.map(a => {
          const live   = hudData[a.id]?.connected
          const coords = agentCoords[a.id]
          return (
            <div key={a.id} className={`c2-roster-card ${live ? 'live' : ''}`}
              style={{ '--agent-color': a.color }}>
              <div className="c2-roster-id" style={{ color: a.color }}>
                {a.id.replace(/([a-z])(\d)/, '$1-$2').toUpperCase()}
              </div>
              <div className={`c2-roster-status ${live ? 'live' : ''}`}>
                {live ? '● ACTIVE' : '○ OFFLINE'}
              </div>
              <div className="c2-roster-coords">
                {coords ? `X ${coords.x.toFixed(1)}  Y ${coords.y.toFixed(1)}` : '-- · --'}
              </div>
            </div>
          )
        })}
      </div>

      {/* Footer stats — bottom-left */}
      <div className="c2-footer-bar">
        <span className="c2-footer-stat">ONLINE {onlineCount} / {AGENTS.length}</span>
        <span className="c2-footer-stat">ROS {status === 'connected' ? 'LIVE' : status.toUpperCase()}</span>
        <span className="c2-footer-stat">C2-OMEGA</span>
      </div>

      {/* Orbit button — bottom-right */}
      <button className="c2-orbit-btn" onClick={toggleOrbit}>
        {orbitMode ? '⟳ ORBIT ON' : '⟳ ORBIT OFF'}
      </button>
    </div>
  )
}
