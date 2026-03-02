# GeoCloud Fleet Simulation — Expert Technical Review
## Reviewer: Marcus Jansen, Principal Engineer, Autonomous Flight Systems & Orbital Dynamics, SpaceX
### Date: 2026-02-21 | Classification: Pre-Investment Technical Due Diligence

---

## 1. Executive Summary — Flight Readiness Rating

**Rating: NOT FLIGHT READY. Demo-viable with significant caveats. Production: 18–24 months of rework minimum.**

I have reviewed every file in this repository. Let me be direct with you the way I would be with my team before a CDR: this system has genuine architectural ambition and the bones of a real edge-robotics platform. The component selection is credible — PX4 SITL, MAVROS, slam_toolbox, Nav2, Zenoh, MediaMTX — these are defensible choices. George O'Connor clearly understands the technology landscape.

However, the implementation contains multiple *show-stopping bugs* that will cause live failures during a demo, at least two of which are in the critical video pipeline path that investors will be watching on their screens. The drone control architecture lacks any meaningful state machine, there is no fault recovery logic anywhere in the codebase, the container stack has a fundamental environmental error that means the video pipeline will produce zero output, and the monitoring system incorrectly labels high-latency frames as "dropped" — meaning the dashboard will show false data throughout the demo.

For a simulation demo in a controlled environment with the system pre-warmed and pre-validated, most of these can be papered over. For a claim to "production-grade" or "flight-ready" architecture, the answer is no. The gaps between what the README promises and what the code delivers are substantial and specific.

I will enumerate every critical issue by file and line. There are no ambiguities in my findings.

---

## 2. Control Systems Analysis — Drone GNC, MAVROS Bridge, Offboard Mode Architecture

### 2.1 Offboard Mode Entry — No State Machine, No Arming Sequence

The drone control is handled by `px4_offboard/offboard_control` launched from `fleet_sim.launch.py` lines 274–286. The parameters are straightforward: circle trajectory, 2m radius, 1.5m altitude, 0.3 rad/s angular rate.

The critical missing piece: there is no explicit arming and mode-switching sequence visible in this codebase. PX4 OFFBOARD mode has a hard prerequisite — the flight controller must receive a continuous stream of setpoints at a minimum of 2 Hz *before* the mode switch is accepted, and it must be maintained continuously. If that stream drops for more than 0.5 seconds, PX4 will exit OFFBOARD and failsafe. The `px4_offboard` package implements this internally, but there is zero monitoring or recovery logic here for the case where that stream is interrupted.

The arming topic `/drone_1/mavros/cmd/arming` is listed in Zenoh publishers (`zenoh_bridge.json5` lines 65–68) — meaning arming commands can be sent from the cloud. This is operationally correct but exposes the arming authority path over an unauthenticated, unencrypted channel. The Zenoh connect endpoint is empty and there is no TLS. You are one misconfigured cloud client away from arming a real vehicle over the internet.

### 2.2 VINS-Fusion Config Mismatch

`fleet_sim.launch.py` line 259: VINS-Fusion is configured with `euroc/euroc_stereo_imu_config.yaml` — the EuRoC dataset configuration. The PX4 Iris SITL has completely different camera intrinsics, stereo baseline, and IMU noise parameters than the EuRoC MAV. Running EuRoC config on an Iris camera model will produce VINS odometry that diverges in seconds. This is not a minor calibration issue — VIO with wrong intrinsics generates accelerating drift that renders the 3D map meaningless. The VINS output feeding into `vins_swarm_fusion_node.py` will be garbage, and the Zenoh-routed `/drone_1/vins/odometry` telemetry will be garbage.

For a demo, VINS will need either a correct Iris config file or the VINS layer must be disabled and replaced with Gazebo ground-truth pose.

### 2.3 Single micro-XRCE-DDS Agent for Two Drones

`fleet_sim.launch.py` lines 389–393: a single `micro-xrce-dds-agent udp4 -p 8888` instance serves both drones. PX4 SITL instances are configured with unique `tgt_system` values (1 and 2), but the XRCE-DDS client inside each SITL instance needs to be configured to connect to that single agent with different client keys. If both SITL instances use the same default client key, their DDS participants will collide on the agent side and one or both drones will fail to exchange topics with ROS2. This requires explicit `--client-key` differentiation, which is not configured here.

### 2.4 Dual Bridge Architecture Conflict

There is an architectural conflict in the drone bridge approach. MAVROS (lines 237–248) connects via MAVLink and uses `tgt_system` to differentiate drones. The `micro-xrce-dds-agent` (line 389) bridges PX4 uXRCE-DDS topics directly to ROS2 DDS. These are two parallel bridge mechanisms. The launch file starts both for drones but does not clearly delineate which topics originate from which bridge. The `px4_offboard` node uses `px4_msgs/TrajectorySetpoint` via DDS-XRCE, while setpoint position commands are routed via `/mavros/setpoint_position/local`. Having both paths active simultaneously creates the potential for duplicate command processing or topic namespace conflicts.

---

## 3. Real-Time Systems Critique

### 3.1 Is ROS2 Appropriate Here?

For a research simulation: yes. For a claim of production autonomous flight software: no. ROS2 with `rmw_fastrtps_cpp` on a standard Linux kernel provides no timing guarantees. The executor model is cooperative, not preemptive. A slow callback in one node — and `_detect_loop_closures` in `vins_swarm_fusion_node.py` is provably slow — will delay every other callback in that executor.

At SpaceX we do not run flight-critical GNC in a framework that cannot provide bounded worst-case execution times. The ROS2 real-time executor exists and `ros2_control` with RT kernels can approach determinism for actuation loops — but none of that is configured here.

### 3.2 The 2 Hz Fusion Timer Is Blocking and Unbounded

`vins_swarm_fusion_node.py` line 104: `self.create_timer(1.0 / self.fuse_rate, self._fuse_and_publish)`. Each call to `_fuse_and_publish` immediately calls `_detect_loop_closures`. With 500 keyframes per drone and 2 drones, `_detect_loop_closures` performs 500 × 500 = 250,000 Euclidean distance computations using `np.linalg.norm` in a nested Python loop. This runs in the ROS2 executor thread, blocking all other callbacks in that node during execution. This is not bounded and will worsen linearly as the keyframe buffer fills.

### 3.3 Clock Domain Mixing

`latency_exporter.py` line 80 uses `self.get_clock().now().nanoseconds` (ROS2 sim clock). Line 90 uses `time.monotonic()` (wall clock) for Hz calculation. In simulation, ROS2 sim time may advance at non-real-time rates. The Hz measurement uses wall-clock intervals but counts messages timestamped in sim time. Under simulation speedup or slowdown, the Hz calculation will be systematically wrong.

---

## 4. Critical Bugs — Specific File:Line References

### BUG-001 — `scripts/ros2_to_rtsp.sh` line 35/39: Variable Expansion Failure in Heredoc

```bash
python3 - <<'PYEOF' |      # line 35: SINGLE-QUOTED heredoc
...
    ['ros2', 'topic', 'echo', '--no-arr', f'${TOPIC}', ...],  # line 39
```

The Python code is inside a `<<'PYEOF'` heredoc. Single-quoted heredocs suppress ALL bash variable expansion. The string `f'${TOPIC}'` — where `$TOPIC` is a bash variable set at line 26 — will never be expanded. Python receives the literal string `${TOPIC}` as the topic argument to `subprocess.Popen`. The `ros2 topic echo` command will be called with `${TOPIC}` as the topic name, fail immediately, and produce no output. FFmpeg receives an empty pipe and exits. This is a full pipeline failure. The video pipeline produces **zero output** as committed.

**Fix:** Pass the topic as an environment variable and read it in Python with `os.environ`:

```bash
TOPIC="/${ROBOT_NS}/camera/compressed" python3 - <<PYEOF
import sys, os, subprocess, re
topic = os.environ['TOPIC']
proc = subprocess.Popen(
    ['ros2', 'topic', 'echo', '--no-arr', topic, '--qos-profile', 'sensor_data'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=False
)
PYEOF
```

Note: unquoting the heredoc delimiter (`PYEOF` instead of `'PYEOF'`) also requires escaping all Python `$`-signs, making the environment variable approach cleaner.

### BUG-002 — `scripts/ros2_to_rtsp.sh` line 52: Filter Logic and Architectural Unsoundness

```python
frame = bytes([int(v) for v in raw if int(v) < 256])
```

`re.findall(r'\d+', line)` returns decimal strings. Values like `1024` or `65535` that appear in the YAML output (timestamps, sizes, sequence numbers) will be parsed as integers greater than 255 and silently dropped, corrupting the H.264 byte stream. The condition should be `if 0 <= int(v) <= 255`, but even that is architecturally wrong. Parsing YAML text output of `ros2 topic echo` to reconstruct a binary H.264 elementary stream is fundamentally unsound. The YAML output has no guaranteed frame boundary alignment. The correct approach is a proper ROS2 subscriber that writes binary `msg.data` bytes directly.

### BUG-003 — `scripts/vins_swarm_fusion_node.py` line 131: Configured Parameter Ignored, Hardcoded Threshold

```python
def _detect_loop_closures(self):
    LC_DIST_THRESH = 1.0  # meters
```

Line 54 declares `loop_closure_threshold` with default `0.5`. Line 59 reads it: `self.lc_threshold = self.get_parameter('loop_closure_threshold').value`. The `launch.py` at line 332 sets it to `0.5`. However, `_detect_loop_closures` at line 131 defines `LC_DIST_THRESH = 1.0` as a local constant and uses that exclusively. `self.lc_threshold` is **never referenced anywhere in the codebase**. The configured parameter has zero effect on system behavior. The actual loop closure threshold is always 1.0 meters, doubling the intended sensitivity and generating spurious closure detections whenever two drones pass within 1 meter of each other.

**Fix:** Replace `LC_DIST_THRESH = 1.0` with `LC_DIST_THRESH = self.lc_threshold`.

### BUG-004 — `scripts/vins_swarm_fusion_node.py` lines 157–159: Loop Closure Detected but Not Applied

The node detects loop closures and logs them but performs no corrective action. No pose-graph optimization is triggered, no drift correction is applied. The 3D point cloud grows with accumulated VIO drift. The `_detect_loop_closures` return value (the `closures` list) is assigned but only used in a log message. This is a placeholder advertised as a functional fusion system.

### BUG-005 — `docker/docker-compose.edge.yml` lines 66–123: FFmpeg Container Has No ROS2

All four `video_broker_*` services use image `linuxserver/ffmpeg:latest`. This image does not contain ROS2, the `ros2` CLI, `rclpy`, or any ROS2 middleware. The script `ros2_to_rtsp.sh` (called at line 78, 94, 107, 120) begins with `source /opt/ros/humble/setup.bash` (script line 30) and calls `ros2 topic echo` (script line 38). Both will fail with "file not found" errors (exit code 127). All four video broker containers crash on startup and enter a `restart: unless-stopped` crash loop. The entire video pipeline — the primary investor-facing output — produces **zero streams**.

**Fix:** Build a custom image that combines ROS2 and FFmpeg:

```dockerfile
# docker/Dockerfile.video_broker
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y ffmpeg && rm -rf /var/lib/apt/lists/*
COPY scripts/ /scripts/
```

### BUG-006 — `docker/docker-compose.edge.yml`: Missing Prometheus Service

`latency_exporter.py` exposes metrics on port 9091. Grafana is configured at port 3000. There is no Prometheus service in `docker-compose.edge.yml` to scrape the exporter and provide a datasource to Grafana. Grafana has no configured datasource. The dashboard displays "No data" for every panel. The monitoring system is entirely non-functional.

**Fix:** Add a Prometheus service and provisioned datasource:

```yaml
prometheus:
  image: prom/prometheus:latest
  container_name: edge_prometheus
  network_mode: host
  volumes:
    - ./config/prometheus.yml:/etc/prometheus/prometheus.yml:ro
  restart: unless-stopped
```

```yaml
# config/prometheus.yml
global:
  scrape_interval: 5s
scrape_configs:
  - job_name: latency_exporter
    static_configs:
      - targets: ['localhost:9091']
```

### BUG-007 — `scripts/latency_exporter.py` line 96: Dropped Counter Semantics Are Wrong

```python
if latency_ms > LATENCY_WARN_MS:
    self.get_logger().warn(...)
    DROPPED.labels(namespace=ns, topic=MONITORED_TOPIC).inc()
```

The metric `ros2_topic_dropped_total` increments on high latency, not on actual dropped frames. A frame received at 151ms latency is counted as dropped even though it was received. A frame that is genuinely dropped by BEST_EFFORT QoS never triggers the callback and is never counted. The metric is simultaneously false-positive and false-negative. Any dashboard built on this metric is measuring the wrong thing.

### BUG-008 — `docker/docker-compose.edge.yml` lines 51–53: Startup Deadlock

```yaml
command: >
  bash -c "
    ...relay commands... &
    ros2 topic echo /edge/tb3_1/camera/compressed --once &&
    wait
  "
```

`ros2 topic echo --once` blocks indefinitely at cold start because no compressed images are published yet (the video brokers have not connected). This means `ros2_core` hangs at startup. The `depends_on: [ros2_core]` on video brokers creates a circular dependency: brokers wait for `ros2_core` health, `ros2_core` health requires edge topics, edge topics require brokers running. This is a startup deadlock.

### BUG-009 — `config/zenoh_bridge.json5` lines 16–19: Cloud Connection Not Configured

```json5
connect: {
  endpoints: [
    /// "tcp/<CLOUD_VM_IP>:7447"   ← uncomment and fill in for cloud
  ],
},
```

The connect endpoint list is empty. The bridge connects to nothing. All command and telemetry topics configured in the allow-list are bridged to nowhere. The cloud connectivity described in the README does not function.

### BUG-010 — `config/fastdds_profile.xml`: Named QoS Profiles Not Applied to Topics

The XML defines `video_writer_profile`, `video_reader_profile`, `cmd_writer_profile`, `cmd_reader_profile`. In ROS2 with FastDDS, named profiles require explicit topic-level mapping via a `<ros2>` section or `ros__qos_overrides` node parameters. Without this mapping, ROS2 nodes use FastDDS defaults for all writer/reader instances. The tuned RELIABLE/BEST_EFFORT QoS split — the entire point of this configuration file — is not active.

**Fix:** Add to `config/fastdds_profile.xml` inside the `<dds>` block:

```xml
<ros2>
  <topic name="/*/camera/compressed">
    <writer profile_name="video_writer_profile"/>
    <reader profile_name="video_reader_profile"/>
  </topic>
  <topic name="/*/cmd_vel">
    <writer profile_name="cmd_writer_profile"/>
    <reader profile_name="cmd_reader_profile"/>
  </topic>
  <topic name="/*/mavros/setpoint_position/local">
    <writer profile_name="cmd_writer_profile"/>
    <reader profile_name="cmd_reader_profile"/>
  </topic>
</ros2>
```

---

## 5. Container Architecture Deep-Dive

The `docker-compose.edge.yml` has a layered failure cascade. Here is what actually happens on `docker compose up`:

1. **`ros2_core` starts** in `network_mode: host`, joins DDS domain 42, starts four relay processes in background. Then it hits `ros2 topic echo /edge/tb3_1/camera/compressed --once` and **blocks indefinitely**.
2. **Video brokers** wait on `depends_on: [ros2_core]`. The healthcheck may or may not pass depending on whether the relay processes published topics before the blocking `--once`. Even if healthy, video brokers crash immediately because `linuxserver/ffmpeg` has no `ros2` binary (BUG-005).
3. **`restart: unless-stopped`** causes the four crashed video brokers to restart, crash, restart — generating CPU load and log noise throughout the demo.
4. **`zenoh_bridge`** runs but connects to nothing (BUG-009). Silent, no error output.
5. **`latency_exporter`** starts correctly but has intermittent data due to the crashed brokers. Prometheus scrape target does not exist (BUG-006).
6. **Grafana** starts with no datasource. Dashboard shows "No data" on every panel.
7. **`rtsp_server`** (MediaMTX) listens correctly on 8554/8889. No publishers connect. Zero streams available.

**Net result on a clean `docker compose up`: zero video streams, empty Grafana dashboard, no cloud connectivity, four services in crash loops.**

---

## 6. Communication Architecture Analysis

### 6.1 DDS QoS Profile Gap

As documented in BUG-010: the FastDDS profile XML is architecturally correct but functionally inert. The `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable causes FastDDS to load the XML and apply the default participant profile for discovery — so multicast and announce-count tuning is active. But named writer/reader profiles require explicit topic-level mapping that is absent. All topics use FastDDS defaults.

### 6.2 Zenoh Routing: Global Map Bandwidth Warning

`zenoh_bridge.json5` lines 53–55 routes `/global_map_2d` and `/global_map_3d` via Zenoh. A `nav_msgs/OccupancyGrid` for a 20m × 20m map at 5cm resolution is ~160 KB per message. At `merging_rate` 0.5 Hz that is 80 KB/s. The 3D PointCloud2 with 1,000 total keyframes at 12 bytes/point is 12 KB at 2 Hz = 24 KB/s. Combined ~104 KB/s is manageable at simulation scale. At real-world deployment scale with larger maps this becomes a WAN bottleneck. The comment "low-frequency; safe to route" requires qualification.

### 6.3 Missing TURN Server

`mediamtx.yml` line 24 configures only STUN (`stun.l.google.com:19302`). STUN handles NAT traversal for simple (full-cone) NAT only. Enterprise and cloud environments typically use symmetric NAT, where STUN will fail to produce working ICE candidates. Without a TURN relay server, WebRTC does not work for any investor accessing the demo from outside the LAN. The README latency target of `<200ms internet` is unreachable without TURN.

### 6.4 No Transport Security on Any Path

RTSP on 8554 is plaintext. WebRTC signaling on 8889 is unencrypted HTTP. Zenoh on 7447 has no TLS configured. The Grafana admin password is `fleet_admin` hardcoded in the compose file. The arming command path (`/mavros/cmd/arming`) is accessible via unauthenticated Zenoh. For LAN demo purposes this is tolerable. For any internet-facing deployment this is a critical security gap.

---

## 7. Video Pipeline Latency Audit

The README claims: "encoder zerolatency ~0ms, transport BEST_EFFORT negligible, edge 0-50ms, WebRTC glass-to-glass <100ms LAN / <200ms internet."

Tracing the actual pipeline:

```
Gazebo render
  → ros_gz_bridge (parameter_bridge)
  → /{ns}/camera/image_raw
  → image_transport republish (H.264 encode, zerolatency)
  → /{ns}/camera/compressed
  → [ros2_core relay] → /edge/{ns}/camera/compressed
  → [video_broker: ros2 topic echo YAML → Python text parse → stdout]  ← BROKEN (BUG-001, BUG-002, BUG-005)
  → FFmpeg stdin (-fflags nobuffer, -c:v copy)
  → RTSP push → MediaMTX → WebRTC → browser
```

**Stage 1 (Gazebo → bridge):** ~1–5ms. Acceptable.

**Stage 2 (H.264 encode):** `preset=veryfast, tune=zerolatency` is the correct configuration. Zerolatency disables B-frames and lookahead. Encoder latency is approximately 1 frame = ~33ms at 30fps. The `g=10` keyframe interval sets worst-case decoder seek at 333ms — this is a decoder cold-start number, not steady-state encoding latency. The "~0ms" encoder claim is aggressive but achievable in practice.

**Stage 3 (ros2 topic echo → Python → FFmpeg):** This stage is architecturally catastrophic independent of the heredoc bug. Even if BUG-001 were fixed, `ros2 topic echo` outputs YAML text where the `data:` field is a decimal integer list representation of the byte array. For a single 720p H.264 frame at 2 Mbps this is on the order of ~8,000 bytes = ~8,000 integers to parse as text per frame at 30fps. This is not a viable real-time path. Additionally, the reconstructed byte stream has no Annex-B start code synchronization, so FFmpeg cannot reliably locate frame boundaries. The stream is both slow to produce and malformed.

**Stage 4 (MediaMTX WebRTC):** MediaMTX's WebRTC implementation is sound and can achieve sub-100ms glass-to-glass on LAN with a good codec stream. This stage is the most credible part of the pipeline.

**Actual achievable latency if bugs were fixed with proper binary relay:** LAN ~80–150ms glass-to-glass. Internet without TURN: connection fails. Internet with TURN: 200–400ms depending on relay geography.

**The correct video bridge** is a proper ROS2 Python subscriber node:

```python
#!/usr/bin/env python3
# scripts/ros2_to_rtsp_node.py
import sys, subprocess, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage

class VideoRelay(Node):
    def __init__(self, ns: str, rtsp_url: str):
        super().__init__(f'video_relay_{ns}')
        self._ffmpeg = subprocess.Popen(
            ['ffmpeg', '-fflags', 'nobuffer', '-flags', 'low_delay',
             '-f', 'h264', '-i', 'pipe:0',
             '-c:v', 'copy', '-an', '-f', 'rtsp',
             '-rtsp_transport', 'tcp', '-muxdelay', '0', rtsp_url],
            stdin=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )
        self.create_subscription(
            CompressedImage, f'/{ns}/camera/compressed', self._cb, qos
        )

    def _cb(self, msg: CompressedImage):
        if self._ffmpeg.stdin:
            try:
                self._ffmpeg.stdin.write(bytes(msg.data))
                self._ffmpeg.stdin.flush()
            except BrokenPipeError:
                self.get_logger().error('FFmpeg pipe closed')
```

This node must run in a container with **both** ROS2 and FFmpeg — which requires the custom Dockerfile described in BUG-005's fix.

---

## 8. State Machine and Failure Mode Analysis

### 8.1 No Failure State Machine Exists Anywhere

I searched the entire codebase. There is no explicit state machine for any agent. There is no defined response to:

- MAVROS connection loss (drone continues OFFBOARD until PX4 exits on its own timeout)
- VINS divergence (fusion node continues publishing corrupt poses forever)
- Nav2 planner failure (explore_lite stalls waiting for a result that never comes)
- DDS participant loss (no mechanism to detect peer failure)
- Camera bridge failure (latency exporter reports 0 Hz, no alert, no recovery)

For a ground robot, these failure modes produce a stuck robot. For a drone in OFFBOARD mode with a lost setpoint stream, PX4's own failsafe triggers after 0.5s and the vehicle holds position or lands. This is the one safety net in the entire system, and it belongs to PX4, not to this codebase.

### 8.2 SLAM + AMCL Conflict for Ground Robots

Ground robots simultaneously run slam_toolbox (which publishes `map → odom` TF) and Nav2 with AMCL (which also publishes `map → odom` TF). Two nodes writing the same TF transform produces a TF tree conflict. The robot's estimated pose will oscillate between the two localization estimates. The correct configuration is either SLAM-only or AMCL-only, not both simultaneously.

### 8.3 No Startup Sequencing

`fleet_sim.launch.py` launches all nodes approximately simultaneously. `px4_offboard` starts before MAVROS establishes a MAVLink connection. `vins_swarm_fusion_node` starts before any drone has produced keyframes, publishing empty point clouds for the first several seconds. `explore_lite` starts before SLAM has produced a costmap. None of this is fatal — most nodes have their own retry logic — but it produces a chaotic startup that makes the system appear unstable for the first 30–60 seconds of any demo.

---

## 9. Specific Actionable Fixes Summary

| Bug | File | Fix Complexity | Demo Blocking |
|-----|------|----------------|--------------|
| BUG-001 Heredoc variable expansion | `scripts/ros2_to_rtsp.sh:35` | Low (env var approach) | **YES** |
| BUG-002 Filter logic and arch unsoundness | `scripts/ros2_to_rtsp.sh:52` | High (rewrite subscriber) | **YES** |
| BUG-003 LC threshold ignored | `vins_swarm_fusion_node.py:131` | Trivial (one line) | No |
| BUG-004 LC not applied | `vins_swarm_fusion_node.py:157` | High (requires g2o/GTSAM) | No |
| BUG-005 FFmpeg container no ROS2 | `docker-compose.edge.yml:66–123` | Medium (Dockerfile) | **YES** |
| BUG-006 No Prometheus service | `docker-compose.edge.yml` | Low (add service + config) | No |
| BUG-007 Wrong dropped semantics | `latency_exporter.py:96` | Low (logic fix) | No |
| BUG-008 Startup deadlock | `docker-compose.edge.yml:51–53` | Low (remove --once) | Partial |
| BUG-009 Zenoh not connected | `zenoh_bridge.json5:16–19` | Low (fill in endpoint) | No |
| BUG-010 FastDDS profiles not applied | `fastdds_profile.xml` | Low (add ros2 section) | No |
| VINS wrong config | `fleet_sim.launch.py:259` | Medium (new calibration) | Partial |
| SLAM+AMCL conflict | `fleet_sim.launch.py:183` | Low (disable AMCL) | Partial |

---

## 10. SpaceX-Grade Upgrade Path

### 10.1 Deterministic Control Loops

Replace Python-based GNC nodes with C++ using RT scheduling:
- Drone setpoint generation: `ros2_control` hardware interface, `SCHED_FIFO` priority 80, CPU affinity pinned to isolated core
- Minimum setpoint publish rate: 10 Hz (5x PX4's minimum for margin)
- Watchdog timer: if setpoint publish misses 3 consecutive cycles, command hold/land via MAVROS emergency service

### 10.2 Explicit Agent State Machines

Implement a hierarchical state machine per agent:

```
DRONE FSM:
INIT → PREFLIGHT_CHECK → ARMING → OFFBOARD_ENTRY → MISSION
  ↓ (any fault at any state)
FAULT → HOLD_POSITION → [operator acknowledge] → {LAND | RTL | RESUME}
```

The state machine must be the single authority for mode commands. No other node publishes to arming or mode topics outside of the FSM.

### 10.3 Replace vins_swarm_fusion with Real Pose Graph Optimization

Replace `vins_swarm_fusion_node.py` with COVINS-G or Swarm-SLAM:
- Actual loop closure: use DBoW3 or NetVLAD descriptor matching, not position proximity
- Actual correction: GTSAM or g2o factor graph optimization on loop closure detection
- Decentralized operation: Swarm-SLAM handles per-agent state without a central aggregator

### 10.4 Proper Container Architecture

Every service needing both ROS2 and another tool requires a purpose-built image. For production, eliminate `network_mode: host` and use bridge networking with explicit DDS multicast group routing. Use `rmw_cyclonedds_cpp` with `CYCLONEDDS_URI` for better container-native DDS performance. Deploy on Kubernetes with resource limits and liveness probes tied to actual functional checks, not `ros2 topic list`.

### 10.5 Security Hardening

- Zenoh: TLS with mutual certificate authentication for all inter-node connections
- MediaMTX: JWT auth enabled before any internet-facing deployment, SRTP for encrypted media
- Arming path: cryptographic command signing, not just topic-level authentication
- Grafana: rotate default password, VPN-only access
- Remove all plaintext credentials from version-controlled config files

### 10.6 Observability

- OpenTelemetry ROS2 instrumentation for per-topic, per-callback metrics with actual bounded worst-case timing
- Distributed tracing across ROS2 and Docker service boundaries
- Alertmanager rules: drone topic silence for >500ms triggers immediate pager alert
- Separate health endpoint from metric endpoint

### 10.7 Hardware-in-the-Loop Validation

The gap between SITL and real hardware requires:
- Correct Iris IMU calibration file for VINS (not EuRoC)
- Hardware timing characterization: measure actual DDS publish-to-receive latency on target edge hardware under load
- Failure injection test matrix: MAVROS loss mid-flight, DDS saturation, network partition, single drone loss — verify safe behavior in every case

---

## Summary Verdict

| Domain | Rating | Primary Blocking Issue |
|--------|--------|----------------------|
| GNC Architecture | 3/10 | No state machine, wrong VINS config, dual-bridge conflict |
| Real-Time Properties | 2/10 | No RT kernel, O(N²) blocking in timer callback, clock domain mixing |
| Container Stack | 1/10 | Video pipeline 100% broken at startup (BUG-001, BUG-005) |
| Communication/DDS | 4/10 | Profiles defined but not applied (BUG-010), Zenoh disconnected (BUG-009) |
| Video Pipeline | 2/10 | Heredoc + wrong container + text-pipe approach = zero streams |
| Monitoring | 3/10 | Wrong drop semantics, no Prometheus, Grafana has no datasource |
| Fault Tolerance | 1/10 | No state machines, no recovery logic anywhere in codebase |
| Security | 2/10 | No auth, no TLS, plaintext admin password, unprotected arming path |

The architecture is credible. The implementation is not demo-ready without fixing at minimum BUG-001, BUG-005, and BUG-006 — proper video bridge node, correct container images with ROS2, and Prometheus service. Those three fixes make the visual demo functional. The deeper issues — state machines, correct VINS calibration, LC threshold bug, clock domain mixing, FastDDS profile mapping — represent the gap between "demo" and "product."

Fix the enumerated bugs before any live investor demonstration. Do not claim flight-readiness or production-grade architecture without the Section 10 upgrades. The bones are sound. The implementation needs to match the ambition.

---

*Report prepared by Marcus Jansen*
*Principal Engineer, Autonomous Flight Systems & Orbital Dynamics*
*SpaceX | Hawthorne, CA*
*February 21, 2026*

*Review scope: All 10 source files. 10 discrete bugs identified with file:line references. Container startup failure cascade fully traced. Video pipeline failure mode fully analyzed.*
