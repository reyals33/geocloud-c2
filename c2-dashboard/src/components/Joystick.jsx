import { useRef, useCallback } from 'react'

// Outer circle diameter and knob diameter in px
const OUTER = 140
const KNOB  = 44
const R     = (OUTER - KNOB) / 2  // max travel radius = 48 px

/**
 * Virtual thumbstick.
 * onChange({ x, y }) — normalized [-1, 1], y is positive = up = forward
 * axisX / axisY — restrict movement to one axis (e.g. altitude slider)
 */
export default function Joystick({ onChange, axisX = true, axisY = true, label }) {
  const outerRef  = useRef(null)
  const knobRef   = useRef(null)
  const activeRef = useRef(null)   // active pointerId
  const centerRef = useRef({ x: 0, y: 0 })

  const moveKnob = (dx, dy) => {
    if (knobRef.current) knobRef.current.style.transform = `translate(${dx}px, ${dy}px)`
  }

  const process = useCallback((cx, cy) => {
    let dx = axisX ? cx - centerRef.current.x : 0
    let dy = axisY ? cy - centerRef.current.y : 0
    const dist = Math.sqrt(dx * dx + dy * dy)
    if (dist > R) { dx = dx / dist * R; dy = dy / dist * R }
    moveKnob(dx, dy)
    // y is inverted: screen up (negative dy) = positive forward
    onChange({ x: axisX ? dx / R : 0, y: axisY ? -dy / R : 0 })
  }, [axisX, axisY, onChange])

  const onDown = useCallback((e) => {
    if (activeRef.current !== null) return
    e.preventDefault()
    activeRef.current = e.pointerId
    outerRef.current.setPointerCapture(e.pointerId)
    const rect = outerRef.current.getBoundingClientRect()
    centerRef.current = { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2 }
    process(e.clientX, e.clientY)
  }, [process])

  const onMove = useCallback((e) => {
    if (activeRef.current !== e.pointerId) return
    process(e.clientX, e.clientY)
  }, [process])

  const onUp = useCallback((e) => {
    if (activeRef.current !== e.pointerId) return
    activeRef.current = null
    moveKnob(0, 0)
    onChange({ x: 0, y: 0 })
  }, [onChange])

  return (
    <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', gap: 6 }}>
      <div
        ref={outerRef}
        className="joystick-outer"
        style={{ touchAction: "none" }} onPointerDown={onDown}
        onPointerMove={onMove}
        onPointerUp={onUp}
        onPointerCancel={onUp}
      >
        <div className="joystick-cross-h" />
        <div className="joystick-cross-v" />
        <div ref={knobRef} className="joystick-knob" />
      </div>
      {label && <span className="joystick-label">{label}</span>}
    </div>
  )
}
