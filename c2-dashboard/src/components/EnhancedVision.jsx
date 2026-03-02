/**
 * EnhancedVision — Lockheed × Northrop R&D tactical optical payload for Drone 2 FPV.
 *
 * Shader pipeline per frame:
 *   1. UNPACK_FLIP_Y_WEBGL — correct HTML-image Y-axis orientation
 *   2. Minimal chromatic aberration (precision optics — barely perceptible)
 *   3. Subject detection: high-saturation / bright pixels (robots, drones) preserved
 *      in natural colour; background gets tactical NV grade
 *   4. Tactical NV colour grade for background: crushed black → cold teal → white-hot
 *   5. High-contrast Sobel edge enhancement (crystalline, single-pixel)
 *   6. Single-step edge corona (precision halo, no painterly bloom)
 *   7. Mil-dot scope reticle: crosshair arms + range rings + mil-dots (GLSL-drawn)
 *   8. Hot-spot shimmer on highlights
 *   9. Tight CRT phosphor scanlines
 *  10. Fine film grain
 *  11. Circular scope barrel vignette + luminous teal lens rim
 */

import { useEffect, useRef } from 'react'

// ── Shaders ───────────────────────────────────────────────────────────────────
const VERT = `
attribute vec2 aPos;
varying   vec2 vUV;
void main() {
  vUV         = aPos * 0.5 + 0.5;
  gl_Position = vec4(aPos, 0.0, 1.0);
}
`

const FRAG = `
precision highp float;

uniform sampler2D uFrame;
uniform vec2      uRes;
uniform float     uTime;
varying vec2      vUV;

float luma(vec3 c) { return dot(c, vec3(0.2126, 0.7152, 0.0722)); }

float sobelAt(vec2 uv, vec2 tx) {
  float tl = luma(texture2D(uFrame, uv + tx*vec2(-1.,-1.)).rgb);
  float tm = luma(texture2D(uFrame, uv + tx*vec2( 0.,-1.)).rgb);
  float tr = luma(texture2D(uFrame, uv + tx*vec2( 1.,-1.)).rgb);
  float ml = luma(texture2D(uFrame, uv + tx*vec2(-1., 0.)).rgb);
  float mr = luma(texture2D(uFrame, uv + tx*vec2( 1., 0.)).rgb);
  float bl = luma(texture2D(uFrame, uv + tx*vec2(-1., 1.)).rgb);
  float bm = luma(texture2D(uFrame, uv + tx*vec2( 0., 1.)).rgb);
  float br = luma(texture2D(uFrame, uv + tx*vec2( 1., 1.)).rgb);
  float gx = -tl - 2.*ml - bl + tr + 2.*mr + br;
  float gy = -tl - 2.*tm - tr + bl + 2.*bm + br;
  return clamp(sqrt(gx*gx + gy*gy) * 5.0, 0.0, 1.0);
}

float hash(vec2 p) {
  return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

void main() {
  vec2  uv   = vUV;
  vec2  tx   = 1.0 / uRes;
  vec2  fc   = (uv - 0.5) * uRes;   // pixels from screen centre
  float dist = length(fc);

  // ── Minimal chromatic aberration (precision optics) ───────────────────────
  float ab   = 0.0004;
  float r    = texture2D(uFrame, uv + vec2( ab, 0.0)).r;
  float g    = texture2D(uFrame, uv               ).g;
  float b    = texture2D(uFrame, uv - vec2( ab, 0.0)).b;
  vec3  base = vec3(r, g, b);
  float lum  = luma(base);

  // ── Subject detection: coloured robots/drones appear in natural colour ────
  // Saturation (HSV-style): how far the colour is from grey
  float cMax = max(base.r, max(base.g, base.b));
  float cMin = min(base.r, min(base.g, base.b));
  float sat  = cMax > 0.001 ? (cMax - cMin) / cMax : 0.0;
  // High saturation → subject. Moderate brightness also contributes.
  float subject = clamp(sat * 3.8 + smoothstep(0.22, 0.58, lum) * 0.30, 0.0, 1.0);

  // Natural crisp image (slight contrast boost, no colour shift)
  vec3  natural = clamp(pow(max(base, vec3(0.0)), vec3(0.88)) * 1.18 - 0.012, 0.0, 1.0);

  // ── Tactical NV grade for background ─────────────────────────────────────
  float c    = clamp(pow(lum, 0.58) * 1.30, 0.0, 1.0);
  vec3  grade = mix(
    mix(vec3(0.00, 0.005, 0.012),
        vec3(0.02, 0.50,  0.44),  clamp(c * 2.8,        0.0, 1.0)),
    vec3(0.88, 1.00, 0.95),       clamp((c - 0.36)*2.5, 0.0, 1.0)
  );
  grade += base * vec3(0.012, 0.028, 0.022);

  // Blend: subjects = natural colour, background = NV grade
  vec3 imageBase = mix(grade, natural, subject * 0.90);

  // ── Sobel edges (sharp, single-step corona) ───────────────────────────────
  float edge = sobelAt(uv, tx);
  float halo = sobelAt(uv, tx * 1.7) * 0.28;
  vec3  col  = imageBase + vec3(0.48, 1.0, 0.84) * (edge * 2.0 + halo);

  // ── Mil-dot scope reticle ─────────────────────────────────────────────────
  float minR    = uRes.y * 0.055;
  float maxR    = uRes.y * 0.210;
  float rng1R   = uRes.y * 0.145;
  float rng2R   = uRes.y * 0.285;
  float milSpac = (maxR - minR) * 0.25;

  // Anti-aliased 1-pixel crosshair arms
  float hArm = clamp(1.2 - abs(fc.y), 0.0, 1.0)
             * step(minR, abs(fc.x)) * step(abs(fc.x), maxR);
  float vArm = clamp(1.2 - abs(fc.x), 0.0, 1.0)
             * step(minR, abs(fc.y)) * step(abs(fc.y), maxR);
  float cdot = 1.0 - smoothstep(0.8, 2.4, dist);

  // Range rings
  float rng1 = 1.0 - smoothstep(0.0, 1.3, abs(dist - rng1R));
  float rng2 = 1.0 - smoothstep(0.0, 1.1, abs(dist - rng2R));

  // Mil-dots: 3 per arm quadrant
  float md = 0.0;
  float d;
  for (int i = 1; i <= 3; i++) {
    d = minR + float(i) * milSpac;
    md = max(md, 1.0 - smoothstep(0.0, 1.7, length(fc - vec2( d,   0.0))));
    md = max(md, 1.0 - smoothstep(0.0, 1.7, length(fc - vec2(-d,   0.0))));
    md = max(md, 1.0 - smoothstep(0.0, 1.7, length(fc - vec2(0.0,  d  ))));
    md = max(md, 1.0 - smoothstep(0.0, 1.7, length(fc - vec2(0.0, -d  ))));
  }

  float reticle = max(max(hArm, vArm), max(cdot, md));
  col  = mix(col, vec3(0.70, 1.0, 0.88), reticle * 0.94);
  col += vec3(0.04, 0.32, 0.24) * (rng1 * 0.50 + rng2 * 0.30);

  // ── Hot-spot shimmer ──────────────────────────────────────────────────────
  float shimmer = 0.5 + 0.5 * sin(uTime * 3.8 + lum * 20.0);
  col += smoothstep(0.78, 1.0, lum) * vec3(0.0, 0.10, 0.08) * shimmer;

  // ── Tight CRT scanlines ───────────────────────────────────────────────────
  col *= 0.90 + 0.10 * sin(vUV.y * uRes.y * 3.14159 * 2.0);

  // ── Film grain ────────────────────────────────────────────────────────────
  col += (hash(uv + mod(uTime * 0.06, 1.0)) - 0.5) * 0.014;

  // ── Scope barrel vignette + luminous lens rim ─────────────────────────────
  float scR    = uRes.y * 0.465;
  float normR  = dist / scR;
  float barrel = smoothstep(1.03, 0.97, normR);
  col *= barrel;
  float rim    = smoothstep(0.93, 0.97, normR) * smoothstep(1.03, 0.97, normR);
  col += vec3(0.0, 0.52, 0.40) * rim * 0.68;

  gl_FragColor = vec4(clamp(col, 0.0, 1.5), 1.0);
}
`

// ── Component ─────────────────────────────────────────────────────────────────
export default function EnhancedVision({ mediaRef }) {
  const canvasRef = useRef(null)

  useEffect(() => {
    const canvas = canvasRef.current
    const gl = canvas.getContext('webgl', { alpha: false, antialias: false })
    if (!gl) { console.warn('EnhancedVision: WebGL not available'); return }

    const mkShader = (type, src) => {
      const s = gl.createShader(type)
      gl.shaderSource(s, src)
      gl.compileShader(s)
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS))
        console.error('Shader compile error:', gl.getShaderInfoLog(s))
      return s
    }
    const prog = gl.createProgram()
    const vert = mkShader(gl.VERTEX_SHADER,   VERT)
    const frag = mkShader(gl.FRAGMENT_SHADER, FRAG)
    gl.attachShader(prog, vert)
    gl.attachShader(prog, frag)
    gl.linkProgram(prog)
    // Detach and delete compiled shader objects — program retains the linked binary
    gl.detachShader(prog, vert); gl.deleteShader(vert)
    gl.detachShader(prog, frag); gl.deleteShader(frag)
    gl.useProgram(prog)

    const buf = gl.createBuffer()
    gl.bindBuffer(gl.ARRAY_BUFFER, buf)
    gl.bufferData(gl.ARRAY_BUFFER,
      new Float32Array([-1,-1, 1,-1, -1,1, 1,1]), gl.STATIC_DRAW)
    const aPos = gl.getAttribLocation(prog, 'aPos')
    gl.enableVertexAttribArray(aPos)
    gl.vertexAttribPointer(aPos, 2, gl.FLOAT, false, 0, 0)

    const uFrame = gl.getUniformLocation(prog, 'uFrame')
    const uRes   = gl.getUniformLocation(prog, 'uRes')
    const uTime  = gl.getUniformLocation(prog, 'uTime')

    const tex = gl.createTexture()
    gl.bindTexture(gl.TEXTURE_2D, tex)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
    gl.uniform1i(uFrame, 0)

    // Flip HTML image Y-axis to match WebGL texture orientation (fixes upside-down)
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, true)

    let animId
    const t0 = performance.now()

    const render = () => {
      animId = requestAnimationFrame(render)
      const media = mediaRef.current
      if (!media) return
      // Support both <img> (.complete) and <video> (.readyState)
      const ready = media.tagName === 'VIDEO'
        ? media.readyState >= 2
        : media.complete && media.naturalWidth > 0
      if (!ready) return

      const { offsetWidth: w, offsetHeight: h } = canvas
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width  = w
        canvas.height = h
        gl.viewport(0, 0, w, h)
        gl.uniform2f(uRes, w, h)
      }

      gl.uniform1f(uTime, (performance.now() - t0) * 0.001)

      gl.bindTexture(gl.TEXTURE_2D, tex)
      try {
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, media)
      } catch (_) {
        return
      }

      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4)
    }
    render()

    return () => {
      cancelAnimationFrame(animId)
      gl.deleteTexture(tex)
      gl.deleteBuffer(buf)
      gl.deleteProgram(prog)
    }
  }, [mediaRef])

  return (
    <canvas
      ref={canvasRef}
      style={{ position: 'absolute', inset: 0, width: '100%', height: '100%', display: 'block' }}
    />
  )
}
