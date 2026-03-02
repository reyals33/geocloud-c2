# GeoCloud Fleet Simulation
## ROS2 Mixed Fleet + Edge Broker for Low-Latency Cloud Control
**GeoCloud Solutions LLC | George O'Connor**

---

## Architecture

```
┌─────────────────────── LOCAL MACHINE ─────────────────────────────┐
│                                                                     │
│   Gazebo Classic                                                    │
│   ├── tb3_1 (TurtleBot3 Burger)                                    │
│   │    ├── slam_toolbox → /tb3_1/map                               │
│   │    ├── Nav2 (AMCL + planner)                                   │
│   │    └── explore_lite (frontier exploration)                      │
│   ├── tb3_2 (TurtleBot3 Waffle)  [same stack]                      │
│   ├── drone_1 (PX4 Iris)                                           │
│   │    ├── VINS-Fusion → /drone_1/vins/odometry                   │
│   │    ├── MAVROS (PX4 ↔ ROS2 bridge)                              │
│   │    └── px4_offboard (circle trajectory)                        │
│   └── drone_2 (PX4 Iris)  [same stack, sysid=2]                    │
│                                                                     │
│   Fusion Layer                                                      │
│   ├── multirobot_map_merge → /global_map_2d                        │
│   └── vins_swarm_fusion   → /global_map_3d                        │
│                                                                     │
│   Video Pipeline (per agent)                                        │
│   ├── ros_gz_bridge  (Gazebo → /ns/camera/image_raw)               │
│   └── image_transport (raw → H.264 /ns/camera/compressed)          │
│                                                                     │
└───────────────────┬───────────────────────────────────────────────┘
                    │ DDS (FastDDS multicast, domain 42)
┌───────────────────▼───────────────────────────────────────────────┐
│                  EDGE BROKER (Docker Stack)                         │
│                                                                     │
│   ros2_core       → subscribes fleet topics, relays to /edge/*     │
│   video_broker_*  → FFmpeg: /ns/camera/compressed → RTSP           │
│   mediamtx        → RTSP → WebRTC (port 8889, sub-100ms)           │
│   zenoh_bridge    → routes cmds/odom/maps to cloud (not video)     │
│   latency_exporter→ Prometheus metrics on :9091                    │
│   grafana         → Fleet health dashboard on :3000                 │
│                                                                     │
└───────────────────┬───────────────────────────────────────────────┘
                    │ WebRTC (video) + Zenoh (commands/telemetry)
┌───────────────────▼───────────────────────────────────────────────┐
│               CLOUD CONSOLE (Browser / Custom App)                  │
│   - Live WebRTC video feeds (all 4 agents)                         │
│   - Odometry / SLAM map visualization (via Zenoh)                  │
│   - Command publishing: cmd_vel, setpoint_position                 │
└────────────────────────────────────────────────────────────────────┘
```

---

## File Structure

```
fleet_sim_project/
├── launch/
│   └── fleet_sim.launch.py          # Main ROS2 launch file (all fleet + stack)
├── config/
│   ├── mediamtx.yml                 # RTSP/WebRTC server config (low-latency tuned)
│   ├── fastdds_profile.xml          # DDS QoS profiles (BEST_EFFORT video, RELIABLE cmd)
│   └── zenoh_bridge.json5           # Zenoh DDS bridge: which topics go to cloud
├── docker/
│   └── docker-compose.edge.yml      # Full edge broker container stack
├── scripts/
│   ├── setup.sh                     # One-shot dependency installer
│   ├── ros2_to_rtsp.sh              # ROS2 compressed → FFmpeg → RTSP bridge
│   ├── latency_exporter.py          # Prometheus metrics exporter
│   └── vins_swarm_fusion_node.py    # Multi-drone VINS pose-graph fusion
└── README.md
```

---

## Quick Start

### Prerequisites
- Ubuntu 22.04 LTS
- 16 GB RAM recommended (sim + SLAM is heavy)
- NVIDIA GPU optional but useful for VINS-Fusion

### 1. Install Everything
```bash
# Clone or copy this project, then:
bash scripts/setup.sh

# Skip optional heavy components:
bash scripts/setup.sh --skip-px4 --skip-vins   # ground robots only, fastest
```

### 2. Set Environment
```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/config/fastdds_profile.xml"

# If PX4 installed:
export PX4_ROOT="${HOME}/PX4-Autopilot"
```

### 3. Launch Simulation
```bash
# Full stack (Nav2 + SLAM + fusion + offboard + exploration + video)
ros2 launch fleet_sim fleet_sim.launch.py

# Headless (no Gazebo GUI — good for servers)
ros2 launch fleet_sim fleet_sim.launch.py headless:=true

# Ground robots only (no drones, faster startup)
ros2 launch fleet_sim fleet_sim.launch.py enable_offboard:=false
```

### 4. Launch Edge Broker
```bash
cd docker/
docker compose -f docker-compose.edge.yml up -d

# View streams:
# WebRTC: http://localhost:8889/tb3_1  (and tb3_2, drone_1, drone_2)
# RTSP:   rtsp://localhost:8554/tb3_1  (VLC or ffplay)
# Dashboard: http://localhost:3000     (Grafana, admin/fleet_admin)
```

---

## Key Topics Reference

| Topic | Type | Purpose |
|---|---|---|
| `/{ns}/camera/image_raw` | `sensor_msgs/Image` | Raw sim camera |
| `/{ns}/camera/compressed` | `sensor_msgs/CompressedImage` | H.264 for streaming |
| `/{ns}/map` | `nav_msgs/OccupancyGrid` | Per-robot 2D SLAM map |
| `/global_map_2d` | `nav_msgs/OccupancyGrid` | Merged ground robot map |
| `/global_map_3d` | `sensor_msgs/PointCloud2` | Fused drone VINS map |
| `/{ns}/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/{ns}/vins/odometry` | `nav_msgs/Odometry` | Drone VIO odometry |
| `/{ns}/cmd_vel` | `geometry_msgs/Twist` | Ground robot control |
| `/{ns}/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Drone position setpoint |

---

## Video Pipeline Detail

```
Gazebo camera plugin
    │  (sensor_msgs/Image via gz transport)
    ▼
ros_gz_bridge (parameter_bridge)
    │  /{ns}/camera/image_raw  (ROS2 topic)
    ▼
image_transport republish
    │  FFmpeg H.264 encoding:
    │    -preset veryfast      (encoder speed > quality; fine for monitoring)
    │    -tune zerolatency     (no B-frames, no lookahead; encoder adds ~0ms delay)
    │    -g 10                 (keyframe every 10 frames = 333ms GOP max)
    │    -bitrate 2Mbps        (720p quality over fleet LAN/cloud)
    │  /{ns}/camera/compressed (sensor_msgs/CompressedImage, H.264 ES)
    ▼
Edge broker: ros2_core (relay) → video_broker (FFmpeg stdin) → MediaMTX RTSP
    ▼
MediaMTX WebRTC server (port 8889)
    │  DTLS-SRTP encrypted; ICE for NAT traversal
    ▼
Cloud console browser (WebRTC JS)
    Glass-to-glass target: < 100ms on LAN, < 200ms over internet
```

---

## Latency Monitoring

```bash
# Per-robot video latency (terminal)
ros2 topic delay /tb3_1/camera/compressed
ros2 topic delay /drone_1/camera/compressed

# Message rate
ros2 topic hz /tb3_1/camera/compressed   # Should be ~30 Hz

# Grafana dashboard (after edge broker up)
# http://localhost:3000 → Fleet Latency Dashboard
```

---

## Scaling Path

| Stage | Config | Notes |
|---|---|---|
| **Local sim** | This repo as-is | ~4 agents, single machine |
| **LAN (real hardware)** | Replace Gazebo spawns with real drivers; same launch structure | Add Raspberry Pi camera nodes |
| **Multi-machine** | Set `ROS_DOMAIN_ID=42` on all machines; enable multicast on switch | Verify DDS discovery with `ros2 multicast receive/send` |
| **Cloud edge** | Deploy `docker-compose.edge.yml` on cloud VM; set Zenoh router endpoint | Point `zenoh_bridge.json5` connect to cloud VM IP |
| **Production scale** | Add Kubernetes; one pod per robot | Use DDS-XRCE or Zenoh for WAN; drop raw DDS over internet |

---

## SLAM Architecture Notes

### Ground Robots (slam_toolbox)
- 2D lidar SLAM; online async mode (maps while moving)
- Each robot builds its own `/ns/map`
- `multirobot_map_merge` fuses into `/global_map_2d` using known spawn poses as initial alignment

### Drones (VINS-Fusion)
- Tightly-coupled stereo VIO with IMU pre-integration
- Each drone outputs `/ns/vins/odometry` and `/ns/vins/keyframe_pose`
- `vins_swarm_fusion_node.py` aggregates keyframes, runs spatial loop closure heuristic, publishes `/global_map_3d` (PointCloud2)
- **For production**: replace with Omni-swarm (decentralized VIO + UWB ranging) or COVINS-G (centralized keyframe graph)

### ORB-SLAM3 Atlas (alternative to VINS)
- Atlas = collection of disconnected sub-maps per agent
- When tracking lost → new sub-map auto-created (no crash/restart)
- Loop/Map Merge thread: DBoW2 place recognition → RANSAC geometric verification → SE(3)/Sim(3) merging with local BA
- Single-agent robustness is excellent; multi-drone requires Swarm-SLAM wrapper

---

## Camera Recommendations (Real Hardware)

| Camera | Cost | Interface | Best For | ROS2 Package |
|---|---|---|---|---|
| Raspberry Pi Camera V3 | ~$25 | CSI | Ground robots, lightweight drones | `v4l2_camera` |
| Logitech C270 | ~$20 | USB/UVC | Ground robots, dev bench | `usb_cam` |
| Intel RealSense D435i | ~$150 | USB3 | Drones (stereo + IMU for VINS) | `realsense2_camera` |
| Arducam OV9281 (stereo pair) | ~$60 total | CSI | Best VINS stereo input on RPi/Jetson | `v4l2_camera` |

For VINS-Fusion on real drones: **Intel RealSense D435i** is the easiest — built-in stereo + IMU, single USB cable, native ROS2 package.

---

## Troubleshooting

**DDS discovery fails (nodes can't see each other)**
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 multicast receive   # Terminal 1
ros2 multicast send      # Terminal 2 — should receive on T1
```

**Gazebo Iris SDF not found**
```bash
ls ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf
# If missing: re-run: make px4_sitl gazebo-classic_iris in PX4-Autopilot/
```

**Video latency too high (>200ms)**
- Reduce GOP: `compressed.g:=5` (trade bandwidth for latency)
- Check encoder: ensure `libx264` or NVENC available in FFmpeg (`ffmpeg -encoders | grep 264`)
- Switch to UDP transport in mediamtx.yml for LAN use

**VINS-Fusion fails to initialize**
- Calibrate camera/IMU extrinsics (use Kalibr or built-in VINS online calibration)
- Ensure IMU topic rate ≥ 200 Hz
- Check that stereo baseline matches config YAML
