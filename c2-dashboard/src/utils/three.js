import * as THREE from 'three'

// ROS coords (x=fwd, y=left, z=up) → Three.js (x, y=up, z=toward-viewer)
export function r2t(x, y, z) {
  return new THREE.Vector3(x ?? 0, z ?? 0, -(y ?? 0))
}

// Quaternion → yaw (rotation around ROS z-axis)
export function quatToYaw({ x, y, z, w }) {
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
}

// Update a vertex-color trail buffer (trail.pts.length determines trail length)
export function shiftTrail(trail, x, y, z) {
  const pts = trail.pts
  const len = pts.length
  for (let i = 0; i < len - 1; i++) pts[i].copy(pts[i + 1])
  pts[len - 1].set(x, y, z)
  trail.count = Math.min(trail.count + 1, len)

  for (let i = 0; i < len; i++) {
    const t  = i / (len - 1)
    const tt = t * t   // squared for sharper bright head
    trail.posArr[i * 3]     = pts[i].x
    trail.posArr[i * 3 + 1] = pts[i].y
    trail.posArr[i * 3 + 2] = pts[i].z
    trail.colArr[i * 3]     = trail.color.r * tt
    trail.colArr[i * 3 + 1] = trail.color.g * tt
    trail.colArr[i * 3 + 2] = trail.color.b * tt
  }
  trail.line.geometry.attributes.position.needsUpdate = true
  trail.line.geometry.attributes.color.needsUpdate    = true
  trail.line.geometry.setDrawRange(0, trail.count)
}

// Dispose all geometries and materials in a Three.js scene
export function disposeScene(scene) {
  scene.traverse(obj => {
    if (obj.geometry) obj.geometry.dispose()
    if (obj.material) {
      if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose())
      else obj.material.dispose()
    }
  })
}
