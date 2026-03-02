# ViDeG — Session Summary & Next Steps
_Last updated: 2026-02-23  |  Author: George O'Connor / GeoCloud Solutions LLC_

---

## What ViDeG Is

A **latency engineering testbed** demonstrating synchronized, low-latency command and
control of a ground robot through commercial cloud infrastructure. The pipeline:

```
Gazebo sim camera
    → JPEG encode (edge)
        → WebSocket transport (WAN via AWS relay)
            → browser video display
                                        ← browser keypress
                                    ← WebSocket command
                                ← edge ROS2 /tb3_2/cmd_vel
                            ← Gazebo DiffDrive response
                        ← /tb3_2/odom velocity confirmation
                    → Cmd RTT chip updates live on console
```

**The claim being proven:** sub-150 ms cmd round-trip, sub-300 ms glass-to-glass video,
at 720p30 over commercial AWS. Every number on the screen is a real measurement.

**Target market:** defense small UGV teams, first responders, federal demonstrations —
where the alternative is either raw ROS2 teleop (no cloud, no scale) or expensive
proprietary platforms with opaque latency.

---

## Current State

### Phase 1 — Local pipeline ✅ COMPLETE
### Phase 2 — WAN deployment ✅ COMPLETE (2026-02-23)

Full end-to-end cloud teleoperation proven. Live demo conducted with remote viewers.

### Measured Baselines

| Path | Cmd RTT | Video latency |
|---|---|---|
| Localhost via relay | **82 ms** | ~1 ms (JPEG decode) |
| EC2 us-west-2 WAN ✅ MEASURED | **125–150 ms** | ~30 ms |

82ms localhost = sim physics dominated (~80ms) + relay overhead (<1ms).
125–150ms WAN = 82ms baseline + ~40–70ms Oregon round trip from demo location.
Real robot (no sim physics delay) expected to drop to ~20–40ms localhost, ~50–80ms WAN.

---

## What's Working

| Component | Status | Detail |
|---|---|---|
| Gazebo Harmonic sim | ✅ | tb3_2 Waffle only; headless RTF ~0.98 |
| ros_gz_bridge | ✅ | camera, cmd_vel, odom, scan all bridged |
| DiffDrive topic fix | ✅ | `_patched_sdf()` scopes cmd_vel AND odom to `/model/tb3_2/` |
| Gazebo unpause | ✅ | t=7s `gz service` force-unpause; GUI race condition solved |
| `videg_console_server.py` | ✅ | WebSocket JPEG stream + JSON telemetry, 30 fps target |
| `web/console.html` | ✅ | D-pad, WASD, speed sliders, video panel, RTT chip |
| `videg_relay.py` | ✅ | Cloud WebSocket broker — deployed and running on EC2 |
| EC2 relay | ✅ | t3.micro us-west-2, Elastic IP 44.253.251.75, screen session 'relay' |
| WAN teleoperation | ✅ | Browser → EC2 → WSL2 → Gazebo → robot moves |
| Remote demo sharing | ✅ | Share console.html as file; viewer opens locally, connects to ws://44.253.251.75:8765 |

---

## Key Bugs Fixed

| Bug | Fix |
|---|---|
| DiffDrive subscribed to bare `/cmd_vel` — robot wouldn't move | `_patched_sdf()`: replaces `<topic>cmd_vel</topic>` → `/model/{ns}/cmd_vel` in temp SDF |
| Odom not publishing to `/model/tb3_2/odom` | DiffDrive uses `<odom_topic>` not `<topic>`; fixed `_patched_sdf()` |
| Gazebo physics frozen at RTF 0.012 | GUI WorldControl sends `pause:true` on connect; fixed with 3s gzclient delay + t=7s force-unpause |
| Zenoh bridge SIGSEGV (exit 139) | `qos_overrides` not supported in zenoh-plugin-ros2dds v1.7.2; removed block |
| WSL2 RTF 0.012 | Ogre2 rendering CPU usage; fixed with `headless:=true` |
| Gazebo camera black / no frames headless | Ogre2 needs display in headless WSL2; fixed with `LIBGL_ALWAYS_SOFTWARE=1` in `start_sim.sh` |
| Edge server not seeing ROS2 topics | `start_edge.sh` missing `export ROS_DOMAIN_ID=42`; added |
| `image_bridge` not starting after sim restart | Must be started manually AFTER sim spawns robot; added `start_image_bridge.sh` |
| Port 8766 already in use on restart | Kill with `kill $(lsof -ti:8766)` before restart |

---

## Quick-Start for Next Session

> ⚠️ IMPORTANT: Always use the bash scripts in `scripts/` — never paste multi-line
> commands directly into terminal. Line breaks cause commands to split and fail.

### Kill old stuff first

```bash
kill $(lsof -ti:8765) 2>/dev/null; kill $(lsof -ti:8766) 2>/dev/null; kill $(lsof -ti:8080) 2>/dev/null
pkill -f "gz " 2>/dev/null; pkill -f "ros2" 2>/dev/null; sleep 2
```

### Startup order (4 terminals)

```bash
# T1 — Relay
bash /home/georg/projects/ViDeG/scripts/start_relay.sh

# T2 — Fleet sim
bash /home/georg/projects/ViDeG/scripts/start_sim.sh
# Wait for: "unpause_physics data: true" (~10s)

# T3 — Image bridge (MUST start AFTER sim spawns robot)
bash /home/georg/projects/ViDeG/scripts/start_image_bridge.sh

# T4 — Edge server (currently points to EC2)
bash /home/georg/projects/ViDeG/scripts/start_edge.sh
```

### EC2 relay (if dead — SSH in and restart)

```bash
ssh -i ~/projects/ViDeG/ViDeG.pem ubuntu@44.253.251.75
source ~/venv/bin/activate
screen -r relay          # reattach if running
# if dead:
screen -S relay
python3 ~/videg_relay.py
# Ctrl-A, D to detach
```

### Browser

- Local: `http://localhost:8080/console.html` → WS: `ws://44.253.251.75:8765`
- Remote demo: share `web/console.html` file; viewer opens locally → same WS URL

### Gazebo visual (optional — see the robot)

```bash
source /opt/ros/jazzy/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -g
```

### Diagnostics

```bash
bash /home/georg/projects/ViDeG/scripts/check_all.sh      # cv2, camera topic info + hz
bash /home/georg/projects/ViDeG/scripts/check_odom.sh     # odom topic info + hz
bash /home/georg/projects/ViDeG/scripts/check_gz.sh       # GZ side topics + data
```

---

## Next Priority — WSS Upgrade (wss://)

**Why needed:** Once console.html is hosted on HTTPS, browsers block plain `ws://`.
WSS is required before a proper public URL exists.

**Setup script ready:** `scripts/setup_ec2_nginx.sh` — run on EC2 after getting a domain.

### Steps

1. **Get a domain**
   - Option A (free, test today): `44-253-251-75.sslip.io` — no signup, works with Let's Encrypt
   - Option B (recommended, ~$12/yr): Namecheap or Route 53; A record → 44.253.251.75

2. **EC2 security group** — add ports 80 and 443; can remove 8765/8766 after

3. **Run setup script on EC2**
   ```bash
   # Copy script to EC2
   scp -i ~/projects/ViDeG/ViDeG.pem \
       ~/projects/ViDeG/scripts/setup_ec2_nginx.sh \
       ubuntu@44.253.251.75:~/

   # SSH in and run
   ssh -i ~/projects/ViDeG/ViDeG.pem ubuntu@44.253.251.75
   bash setup_ec2_nginx.sh YOUR_DOMAIN YOUR_EMAIL
   ```

4. **Upload console.html to EC2**
   ```bash
   scp -i ~/projects/ViDeG/ViDeG.pem \
       ~/projects/ViDeG/web/console.html \
       ubuntu@44.253.251.75:~/videg-web/
   ```

5. **Update local files** (ask Claude to do this once domain is known)
   - `start_edge.sh`: `--cloud wss://YOUR_DOMAIN/ws-edge`
   - `console.html`: default WS URL → `wss://YOUR_DOMAIN/ws-browser`

6. **Restart relay on localhost only**
   ```bash
   python3 ~/videg_relay.py --host 127.0.0.1
   ```

**Result:** Anyone opens `https://YOUR_DOMAIN/console.html` from anywhere, fully secure.

---

## Architecture Reference

### Current (Phase 2 — working)

```
Browser (anywhere, file:// or localhost)
    │  ws://44.253.251.75:8765
    ▼
EC2 us-west-2  videg_relay.py  (44.253.251.75)
    │  ws://44.253.251.75:8766
    ▼
WSL2 edge  videg_console_server.py --cloud ws://44.253.251.75:8766
    │  ROS2 DDS (domain 42)
    ▼
Gazebo Harmonic  tb3_2 Waffle
```

### After WSS upgrade

```
Browser (anywhere)
    │  wss://YOUR_DOMAIN/ws-browser
    ▼
nginx (EC2)  ← TLS termination, serves console.html
    │  ws://127.0.0.1:8765
    ▼
videg_relay.py (localhost only)
    │  ws://127.0.0.1:8766  ←  wss://YOUR_DOMAIN/ws-edge
    ▼
Edge (WSL2 or Pi 5 or Jetson)
    │  ROS2
    ▼
Robot
```

### Real hardware path (Pi 5 / Jetson — same code)

```
Pi 5 on WiFi anywhere
    ├── videg_console_server.py --cloud wss://YOUR_DOMAIN/ws-edge
    │       publishes /cmd_vel, subscribes to /camera/image_raw
    └── motor driver node  ← subscribes /cmd_vel → real motors
```
No changes to cloud or browser. Pi just replaces WSL2 in the relay slot.

---

## Roadmap

| Priority | Task | Status |
|---|---|---|
| 1 | WSS — nginx + Let's Encrypt on EC2 | 🔜 Next |
| 2 | Add robot 2 (uncomment tb3_1 in FLEET) | Ready to do |
| 3 | WebRTC video path (lower latency on congested WAN) | Future |
| 4 | Nav2 goal button (click waypoint → autonomous nav) | Future |
| 5 | Real edge hardware — Pi 5 or Jetson Orin Nano 8GB (~$500) | Budget pending |
| 6 | Fleet relay redesign — N robots, per-robot ID routing | Future |

---

## Protocol Stack

| Path | Protocol | Encrypted | Status |
|---|---|---|---|
| Browser → Relay | WebSocket ws:// | ❌ | Working |
| Edge → Relay | WebSocket ws:// | ❌ | Working |
| Video frames | JPEG over WebSocket | ❌ | Working |
| Commands | JSON over WebSocket | ❌ | Working |
| ROS2 internal | DDS UDP | ❌ (LAN only) | Working |
| Browser → Relay | WebSocket wss:// | ✅ | Next step |

---

## File Map

```
ViDeG/
├── launch/
│   └── fleet_sim.launch.py        v2.4.1  Gazebo Harmonic + tb3_2 Waffle
├── scripts/
│   ├── videg_console_server.py    v1.1    WebSocket bridge; --cloud flag; RTT measurement
│   ├── videg_relay.py             v1.0    Cloud WebSocket broker — on EC2 44.253.251.75
│   ├── setup_ec2_nginx.sh                 EC2 nginx + WSS setup script (run on EC2)
│   ├── start_relay.sh                     T1 launcher
│   ├── start_sim.sh                       T2 launcher (has LIBGL_ALWAYS_SOFTWARE=1)
│   ├── start_image_bridge.sh              T3 launcher (run AFTER sim)
│   ├── start_edge.sh                      T4 launcher (→ ws://44.253.251.75:8766)
│   ├── check_all.sh                       cv2 + camera diagnostics
│   ├── check_odom.sh                      odom topic diagnostics
│   └── check_gz.sh                        Gazebo side topic diagnostics
├── web/
│   └── console.html               v1.1    Browser UI — shareable as standalone file
├── docker/
│   └── docker-compose.edge.yml    v2.1.0  ROS2 Jazzy edge stack (not used in current path)
└── .venv/                                 pip: websockets, opencv-python-headless, numpy
```

## Key Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/tb3_2/camera/image_raw` | `sensor_msgs/Image` | GZ→ROS | Video feed |
| `/tb3_2/cmd_vel` | `geometry_msgs/Twist` | ROS→GZ | Drive commands |
| `/tb3_2/odom` | `nav_msgs/Odometry` | GZ→ROS | RTT measurement + velocity display |
| `/tb3_2/scan` | `sensor_msgs/LaserScan` | GZ→ROS | SLAM input (when enabled) |
| `/clock` | `rosgraph_msgs/Clock` | GZ→ROS | Sim time |

---

## Session Changelog

| Date | Changes |
|---|---|
| 2026-02-21 | Initial build: launch file, console server, console UI, Docker stack |
| 2026-02-21 | Fixed: DiffDrive topic scoping, Gazebo unpause race, RTF 0.012 on WSL2 |
| 2026-02-21 | Fixed: Zenoh SIGSEGV, GID mismatch, Nav2 container namespace collision |
| 2026-02-21 | Upgraded Docker stack ROS Humble → Jazzy |
| 2026-02-22 | Added Cmd RTT measurement (odom response timing) + odom velocity display |
| 2026-02-22 | Wrote `videg_relay.py` cloud broker + `--cloud` flag on edge server |
| 2026-02-22 | AWS region decided: us-west-2 (Oregon) |
| 2026-02-22 | Local relay pre-flight PASSED — Cmd RTT 82ms, video 1ms through relay |
| 2026-02-22 | Fixed: odom SDF tag `<odom_topic>` — patched in `_patched_sdf()` |
| 2026-02-22 | Fixed: `start_edge.sh` missing `ROS_DOMAIN_ID=42` |
| 2026-02-22 | Fixed: `LIBGL_ALWAYS_SOFTWARE=1` required for Gazebo camera headless |
| 2026-02-23 | EC2 deployed: t3.micro us-west-2, Elastic IP 44.253.251.75 |
| 2026-02-23 | WAN Cmd RTT MEASURED: 125–150ms ✅ Proof of concept complete |
| 2026-02-23 | `start_edge.sh` updated → `ws://44.253.251.75:8766` |
| 2026-02-23 | `setup_ec2_nginx.sh` written — WSS upgrade ready to execute |
| 2026-02-23 | Live demo conducted with remote viewers via shared console.html file |
| 2026-02-23 | Confirmed Pi 5 / Jetson can replace WSL2 edge — same code, no cloud changes |
| 2026-02-23 | Fleet scaling architecture discussed — N robots needs relay redesign |
