# GeoCloud C2 Dashboard — Technical Overview
**Author:** George O'Connor  |  **Date:** 2026-03-02  |  **Stack:** React 18 · Three.js · ROS2 Jazzy · Gazebo Harmonic · WebRTC

---

## What It Is

A real-time command-and-control dashboard for a four-agent heterogeneous fleet (2 ground robots, 2 drones) simulated in Gazebo Harmonic on AWS EC2. The browser is the only client — no native app, no proprietary middleware.

**Live URL:** `http://44.253.251.75` (EC2 us-west-2, Elastic IP)

---

## Architecture

```
Gazebo Harmonic (EC2, headless EGL)
  ├─ /robot1,2/camera/image_raw  ──► GStreamer nvh264enc/x264enc
  ├─ /drone1,2/camera/image_raw  ──► GStreamer nvh264enc/x264enc
  │       └─ webrtcbin ◄──► signaling server (ws :8765) ◄──► RTCPeerConnection
  └─ /*/odom, /*/cmd_vel  ◄──► rosbridge_server (ws :9090) ◄──► roslib.js
                                                                    │
                                                           React C2 Dashboard
```

**Five pages:** `/robot1`, `/robot2`, `/drone1`, `/drone2` (live video + telemetry + controls), `/overview` (cinematic Three.js top-down view of all agents).

---

## Key Engineering Highlights

### WebRTC Video Pipeline
- **GStreamer producer** (`gst_webrtc_agent.py`) runs one per agent on EC2: ROS2 `sensor_msgs/Image` → `appsrc` → `nvh264enc` (NVENC) or `x264enc` fallback → `rtph264pay` → `webrtcbin`
- **Trickle ICE** end-to-end: offer sent immediately, candidates forwarded individually via a 145-line Python signaling server (`signaling.py`)
- **`useWebRTC` hook** manages the full lifecycle: WebSocket → offer/answer exchange → `RTCPeerConnection` → `<video>` element, with automatic reconnect and live FPS measurement via `requestVideoFrameCallback`

### Three.js Overview Scene (`/overview`)
- Cinematic camera orbit around a lit arena grid; per-agent 3D meshes (box for ground robots, sphere for drones) interpolated in real time from rosbridge odometry at 10 Hz
- Custom GLSL ground shader with Fresnel-edge glow; CSS2DRenderer agent labels; animated thrust rings on drones
- Spawn-offset math: Gazebo odom is frame-relative; world positions reconstructed by adding SDF spawn poses from `config.js`

### Enhanced Vision Overlay (Drone 2)
- Toggleable WebGL post-processing on the live video feed: NV tactical colour grade, Sobel edge detection, mil-dot reticle, scanlines, film grain, barrel vignette — 154 lines of GLSL, zero external dependencies

### Controls Interaction Model
Three distinct interaction modes on a single button: **tap** (500 ms burst), **hold** (continuous while pressed), **double-tap** (latched continuous until tapped again). Drives both differential-drive ground robots and drone VelocityControl via `/*/cmd_vel` Twist messages.

### ROS2 / Gazebo Integration
- ros_gz_bridge with 12 topic bindings across all four agents
- `image_transport republish` pipeline: Gazebo `sensor_msgs/Image` → JPEG compressed → GStreamer appsrc
- Net RTT measured independently of sim clock: rosapi `/get_time` ping every 2 s

---

## Performance (t3.medium, 4 software-rendered cameras)

| Metric | Value |
|---|---|
| WebSocket (WS) RTT, EC2 | 40–70 ms |
| WebRTC video latency | < 200 ms glass-to-glass |
| Camera frame rate | 5 fps (CPU-limited; NVENC path targets 30 fps) |
| Gazebo real-time factor | ~0.95 |
| EC2 CPU load | ~95% (acceptable for demo) |

---

## Code Structure

```
c2-dashboard/src/
  hooks/    useWebRTC.js · useAgent.js
  components/ VideoFeed · EnhancedVision · MiniMap · OverviewScene · ControlPanel · TelemetryPanel
  pages/    AgentDashboard · OverviewDashboard
  config.js (single source of truth — agents, URLs, speeds)
webrtc/
  gst_webrtc_agent.py   GStreamer WebRTC producer (ROS2 → H.264 → webrtcbin)
  signaling.py          WebSocket signaling server (145 lines, zero deps beyond websockets)
gazebo/
  c2_arena.sdf          4-agent world (ground planes, obstacles, cameras)
  bridge.yaml           ros_gz_bridge topic map
```

---

## Relevance to MoveIt Pro / PickNik

| PickNik Domain | Demonstrated Here |
|---|---|
| React operator interfaces | Full SPA with real-time 3D viz, video, telemetry, controls |
| ROS2 topic handling | roslib subscriptions, Twist publishing, nav_msgs/Odometry parsing |
| Low-latency media | WebRTC H.264, trickle ICE, GStreamer pipeline engineering |
| Gazebo simulation | Headless Harmonic on EC2, ros_gz_bridge, sensor_msgs/Image |
| Three.js 3D rendering | Orbit scene, GLSL shaders, CSS2D labels, per-frame lerp |
