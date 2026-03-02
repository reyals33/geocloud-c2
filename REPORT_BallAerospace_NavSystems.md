# Navigation Systems Technical Assessment Report
## GeoCloud Fleet Simulation — Mixed-Fleet ROS2 Navigation Architecture

**Prepared by:** Dr. Eleanor Vasquez, Distinguished Scientist, Navigation Systems
**Organization:** Ball Aerospace & Technologies Corp.
**Report Classification:** Technical Review — Investor Demo Readiness
**Project:** GeoCloud Fleet Simulation v2.0.0, GeoCloud Solutions LLC / George O'Connor
**Review Date:** 21 February 2026
**Document ID:** BA-NAV-2026-0221-GCS

---

## Preface

This assessment was conducted at the request of the project team in preparation for a high-budget investor demonstration. I have reviewed the complete source code corpus: `fleet_sim.launch.py`, `vins_swarm_fusion_node.py`, `fastdds_profile.xml`, `zenoh_bridge.json5`, `mediamtx.yml`, `docker-compose.edge.yml`, `latency_exporter.py`, and `setup.sh`. My evaluation applies the same standards I have applied to navigation systems on the James Webb Space Telescope, the Kepler Space Observatory, and DoD constellation programs reviewed under AFRL and NRO auspices. The findings are unambiguous and require urgent engineering attention before any investor-facing demonstration.

---

## 1. Executive Assessment — Navigation Systems Readiness and Architecture Verdict

**Overall Navigation Readiness: NOT DEMO-READY in current state.**

The GeoCloud Fleet Simulation presents a coherent architectural vision — a mixed-fleet autonomy stack spanning ground robots and aerial platforms, unified through a multi-modal SLAM fusion layer and routed to the cloud via a DDS/Zenoh/WebRTC pipeline. The vision is technically sound and commercially interesting. The execution contains multiple critical-severity defects that will cause categorical failures during a live demonstration.

The most severe finding is that **the drone navigation subsystem will not function at all**. The VINS-Fusion configuration references a stereo camera pair that does not exist in the PX4 Iris SDF model. The IMU topic routing is broken. The configuration YAML is calibrated for the EuRoC dataset's Leica MoCap-calibrated ASL camera rig, not the Gazebo Iris sensor suite. The result is that VINS-Fusion will either crash on initialization or produce unbounded drift from a miscalibrated, monocular-only input in a framework expecting stereo. Either outcome is fatal to the drone navigation demonstration.

The ground robot stack is structurally more sound but contains a fundamental semantic conflict: Nav2 is launched with a static pre-built map (`map.yaml`) while `slam_toolbox` simultaneously builds a live occupancy grid. AMCL, the Monte Carlo localizer within Nav2, requires a consistent, static reference map to compute particle weights. Operating against a map that is growing in real time will produce localization divergence.

The swarm fusion node (`vins_swarm_fusion_node.py`) implements no actual pose-graph optimization. It collects position triples, runs a brute-force O(N×M) distance comparison labeled "loop closure," and publishes a raw PointCloud2 with no correction applied to any keyframe. It is a data aggregator, not a fusion algorithm.

---

## 2. SLAM Architecture Analysis

### 2.1 Ground Robot SLAM: slam_toolbox (2D)

The selection of `slam_toolbox` in `online_async` mode is well-considered for the TurtleBot3 platform. The `online_async` solver (Sparse Pose Adjustment, as implemented by Kohlbrecher et al. and later extended by Macenski et al. in "SLAM Toolbox: SLAM for the Dynamic World," JOSS 2021) correctly separates the scan-matching thread from the pose-graph optimization thread, preventing scan callback blocking on large map updates. This is architecturally appropriate for a Gazebo simulation where computational resources are shared.

The use of `known_init_poses=True` in `multirobot_map_merge` and reliance on spawn coordinates as initial alignment are correct for a simulation context where ground truth poses are available.

**Identified configuration issue:** The `slam_params_file` is not overridden in the launch file — `mapper_params_online_async.yaml` is used as shipped with `slam_toolbox`. The default `max_laser_range: 20.0` meters significantly exceeds the TurtleBot3 Burger LDS-01 sensor's nominal 3.5m range. Operating SLAM with a configured laser range beyond actual sensor range causes the SPA loop closure search to consider geometrically inconsistent scan pairs, inflating the pose graph unnecessarily.

### 2.2 Drone SLAM: VINS-Fusion (3D VIO)

VINS-Fusion (Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator," IEEE T-RO 2018; Qin et al., "VINS-Fusion: An Optimization-Based Multi-Sensor State Estimator," arXiv 2019) implements tightly-coupled visual-inertial odometry using a sliding-window nonlinear optimization over visual reprojection errors and IMU pre-integration constraints. The backend uses Ceres Solver for the nonlinear least-squares problem. Marginalization is handled via the Schur complement to maintain bounded complexity. This is a mature, well-validated algorithm appropriate for the stated use case. The configuration and sensor routing in this project, however, render the algorithm inoperable. See Sections 4 and 7.

---

## 3. Sensor Fusion Deep-Dive: `vins_swarm_fusion_node.py` Forensic Analysis

### 3.1 Orientation Information is Silently Discarded

In `_kf_callback` (line 115 of `scripts/vins_swarm_fusion_node.py`), the keyframe pose is stored as:

```python
self.keyframes[ns].append((p.x, p.y, p.z, t))
```

The quaternion `msg.pose.orientation` is unconditionally discarded. Proper pose-graph fusion requires SE(3) poses, not R^3 positions. The loss of orientation also means the published `PoseArray` messages always contain identity quaternions (`w=1.0`), rendering them geometrically meaningless for any downstream consumer.

### 3.2 Loop Closure Detection Is a Distance Heuristic With No Geometric Verification

The `_detect_loop_closures` method computes Euclidean distance between position triples across all keyframe pairs. Three critical defects:

**First**, the declared ROS parameter `loop_closure_threshold` (set to 0.5m in the launch file) is never used. The hardcoded local constant `LC_DIST_THRESH = 1.0` at line 131 silently overrides it.

**Second**, spatial proximity is not a valid loop closure criterion in a VIO system. A loop closure requires geometric verification — at minimum, RANSAC-based essential matrix computation over matched image descriptors (e.g., DBoW3 bag-of-words retrieval as used in ORB-SLAM3, or NetVLAD for place recognition). Two drones at the same spatial coordinates may have visited from entirely different headings with no image correspondence.

**Third**, the method's return value is consumed only by a logger statement. No pose-graph correction is applied. The detection result is discarded after logging. "Loop closure detected" in the logs is a false claim.

### 3.3 Computational Complexity at 2 Hz

With `max_keyframes_per_drone=500`, the inner loop executes 500 × 500 = 250,000 Euclidean distance computations per call to `_detect_loop_closures`, invoked at 2 Hz: 500,000 distance evaluations per second. This scales quadratically with `max_keyframes_per_drone`. A `scipy.spatial.cKDTree` would reduce this to O(N log N).

### 3.4 No Coordinate Frame Transformation

Each VINS-Fusion instance initializes its VIO world frame independently at first successful initialization. These frames differ by an unknown rigid body transformation T ∈ SE(3). Concatenating keyframes from two independently-referenced frames into a single `map`-framed PointCloud2 produces a geometrically incorrect global map that appears plausible in RViz but is metrically invalid.

### 3.5 No Uncertainty Propagation

VINS-Fusion's sliding-window optimizer maintains a covariance estimate for the current state. None of this is propagated through the fusion node. The `global_map_3d` PointCloud2 contains raw FLOAT32 XYZ with no associated covariance. Any downstream consumer has no basis for understanding reliability of individual map points.

### 3.6 Monotonically Growing Map with No Management

The keyframe buffer is capped at 500 per drone via FIFO trim, but the published PointCloud2 grows monotonically at every fusion tick. With TRANSIENT_LOCAL QoS durability, the DDS middleware caches the full cloud for late-joining subscribers — an unbounded memory growth vector in the DDS layer.

---

## 4. Critical Sensor and Topic Mismatches

### 4.1 Stereo Camera Hardware Does Not Exist: Fatal

The PX4 Iris SDF defines a single forward-facing camera plugin. It does not define a stereo camera pair. VINS-Fusion is configured with `euroc_stereo_imu_config.yaml` specifying two camera inputs (`cam0`, `cam1`). The remapping in `fleet_sim.launch.py` line 263:

```python
('/vins_fusion/right_image', f'/{ns}/camera/right/image_raw'),
```

references a topic that will never have a publisher. The `ros_gz_bridge` only bridges `/{ns}/camera/image_raw` (single camera). VINS-Fusion in stereo mode will block indefinitely at the stereo initialization barrier. **No VIO output will ever be produced for either drone.**

### 4.2 IMU Topic Routing is Broken: Fatal

The VINS-Fusion remapping in `fleet_sim.launch.py` line 262:

```python
('/vins_fusion/imu', f'/{ns}/imu/data'),
```

is incorrect. MAVROS publishes IMU data on `/{ns}/mavros/imu/data` (hardcoded in `mavros/plugins/imu.cpp`). No node publishes to `/{ns}/imu/data`. Without IMU measurements, VIO initialization is impossible — the IMU pre-integration chain has no input, and the initial gravity vector and bias cannot be estimated.

### 4.3 EuRoC Calibration Applied to Iris Sensor Suite: Operationally Invalid

`euroc_stereo_imu_config.yaml` contains camera intrinsics and IMU-camera extrinsics calibrated for the ETH Zurich EuRoC dataset (Burri et al., "The EuRoC micro aerial vehicle datasets," IJRR 2016): Aptina MT9V034 grayscale camera (752×480), ~110mm stereo baseline, ADIS16448 IMU at 200 Hz. The Gazebo Iris model uses entirely different simulated sensor parameters (640×480, zero distortion, different IMU noise characteristics). Applying the EuRoC calibration introduces systematic reprojection errors in the visual residuals. Even with correct topic routing, the Ceres optimizer would solve against a grossly incorrect measurement model, producing biased, rapidly-drifting state estimates.

---

## 5. Multi-Robot Map Fusion Correctness

### 5.1 Namespace Topic Resolution: Silent Failure Mode

`slam_toolbox` is launched inside a `GroupAction` with `PushRosNamespace(ns)`. When the launch file also passes `slam_toolbox/namespace: ns` as a parameter, a double-namespace condition can arise where the map is published on `/tb3_1/tb3_1/map`. The `map_merge` node subscribing to `/tb3_1/map` would receive no data. The global map would never be populated — silently, with no error logs, because `map_merge` will simply publish an empty `OccupancyGrid`.

### 5.2 Known Init Poses Parameters Not Passed

`multirobot_map_merge` with `known_init_poses=True` requires per-robot initial pose parameters (`robot1_init_pose_x`, `robot1_init_pose_y`, `robot1_init_pose_yaw`, etc.) as ROS node parameters. The launch file sets the flag but does not pass the spawn coordinates of tb3_1 (x=-2.0, y=0.0, yaw=0.0) and tb3_2 (x=2.0, y=0.0, yaw=1.57) as the required parameters. Without them, `map_merge` falls back to unknown-pose estimation mode (image-based alignment), which may fail for maps with insufficient overlap at merge time.

---

## 6. Navigation Stack Integration: Nav2 Static vs. SLAM Live Map Conflict

For each ground robot, the launch file simultaneously starts `slam_toolbox` (online async, publishes `map → odom` TF) and Nav2 with `map.yaml` (static map, AMCL also publishes `map → odom` TF). Two failure modes:

**Failure Mode A — Dual TF Publishers:** Both AMCL and `slam_toolbox` publish the `map → odom` transform. This creates a TF race condition. The TF tree will contain conflicting transforms for the same frame pair, causing `tf2::LookupException` or extrapolation errors in Nav2's costmap, planner, and controller server. Navigation goals will be rejected or the robot will execute erratic trajectories.

**Failure Mode B — Map Inconsistency:** AMCL localizes the robot by comparing laser scans against the loaded static `map.yaml`. As the robot explores areas not in `map.yaml`, AMCL receives unmatched scan returns. The particle weights collapse, the particle filter degenerates, and AMCL publishes a high-covariance, effectively random localization estimate. The Nav2 planner, using a now-invalid robot pose, plans paths to incorrect positions.

**Correct configuration:** Either (a) use `slam_toolbox` as the sole localizer by disabling AMCL and configuring Nav2 to use the live SLAM TF, or (b) disable `slam_toolbox` and use AMCL against the static map. Running both simultaneously serves neither purpose.

---

## 7. IMU Pre-Integration and VIO Initialization Concerns for the Iris Model

### 7.1 IMU Pre-Integration Chain Under Simulation Load

VINS-Fusion implements IMU pre-integration following Forster et al. ("On-Manifold Preintegration for Real-Time Visual-Inertial Odometry," IEEE T-RO 2017), accumulating delta position, velocity, and rotation on the SO(3) manifold with bias correction. Running four agents simultaneously in Gazebo Classic on a single machine may not sustain the required 200+ Hz IMU publication rate without message gaps. VINS-Fusion is sensitive to IMU gaps — a gap of more than a few keyframe intervals breaks the pre-integration chain and requires reinitialization.

### 7.2 Offboard Control Conflicts With VIO Initialization Window

The `px4_offboard` node immediately commands a circular trajectory (`angular_rate=0.3 rad/s`). VINS-Fusion requires approximately 5–10 seconds of controlled motion with adequate feature visibility for initialization — sufficient visual feature tracks (minimum ~50 tracked points across 8+ frames), IMU excitation for gravity vector estimation, and a low-motion period for gyroscope bias estimation. An immediate aggressive circle maneuver before VIO initialization will cause initialization failure.

### 7.3 Single XRCE Agent Fragility

The micro-XRCE-DDS agent is launched as a single process on UDP port 8888 serving both drones. Under Gazebo Classic, namespace isolation between PX4 SITL instances via uXRCE has known race conditions during simultaneous client reconnection after agent restart. This is a fragility point requiring careful validation.

---

## 8. Recommended Navigation Architecture for Production

### 8.1 Ground Robot Navigation

Retain `slam_toolbox` + Nav2 with fix: disable AMCL, configure Nav2 to use `slam_toolbox`-provided `map → odom` TF exclusively. For production, consider **RTAB-Map** (Labbé & Michaud, "RTAB-Map as an Open-Source Lidar and Visual Graph-Based SLAM Library," JFR 2019) for loop closure with multi-session mapping, and **Nav2 Coverage Planner** for structured area coverage.

### 8.2 Drone Visual-Inertial Odometry

**Immediate fix:** Switch VINS-Fusion to monocular mode using the single Iris forward camera with a custom-calibrated `iris_gazebo_mono_config.yaml`. Use **Kalibr** (Furgale et al., "Unified temporal and spatial calibration for multi-sensor systems," IROS 2013) for camera-IMU extrinsic calibration against actual Gazebo sensor parameters.

**Production path:** Replace VINS-Fusion with **ORB-SLAM3** (Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM," IEEE T-RO 2021) in stereo-inertial mode. ORB-SLAM3's Atlas framework provides automatic sub-map creation on tracking loss (no catastrophic restart), DBoW2-based place recognition, and Sim(3) map merging. For multi-drone operation, wrap with **Swarm-SLAM** (Lajoie & Beltrame, "Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems," IEEE RA-L 2024).

### 8.3 Multi-Drone Swarm Fusion

Replace `vins_swarm_fusion_node.py` entirely with one of:

**Option A — Decentralized:** **Omni-Swarm** (Xu et al., "Omni-Swarm: A Decentralized Omnidirectional Visual-Inertial-UWB State Estimator for Aerial Swarm," IEEE T-RO 2022). Decentralized VIO with UWB inter-robot ranging for metric scale anchoring. No central server required. Graceful individual drone dropout.

**Option B — Centralized:** **COVINS-G** (Schmuck et al., "COVINS-G: A Generic Back-End for Collaborative Visual-Inertial SLAM," ICRA 2023). Centralized keyframe graph with full pose-graph optimization (g2o backend), inter-agent loop closure, and covariance propagation.

### 8.4 WAN Map Bandwidth

Replace raw OccupancyGrid streaming over Zenoh with delta compression (transmit only changed cells) or a web-tile-based map rendering service (serve PNG tiles via HTTP rather than routing the raw ROS message). Target: <100 KB/s for map data on WAN.

---

## 9. Enumerated Bug Catalog

**BUG-001 | CRITICAL | `launch/fleet_sim.launch.py`, line 263**
**Title:** Right stereo camera topic has no publisher — VINS-Fusion cannot initialize
The remapping `'/vins_fusion/right_image' → f'/{ns}/camera/right/image_raw'` references a topic that is never published. The PX4 Iris SDF defines exactly one forward camera. VINS-Fusion in stereo mode will block indefinitely at the initialization barrier. No VIO output will ever be produced for either drone.
**Fix:** Add second camera plugin to Iris SDF, or switch VINS-Fusion to monocular mode.

---

**BUG-002 | CRITICAL | `launch/fleet_sim.launch.py`, line 262**
**Title:** IMU topic remapping points to non-existent topic — VIO IMU input absent
`'/vins_fusion/imu' → f'/{ns}/imu/data'` is incorrect. MAVROS publishes on `/{ns}/mavros/imu/data`. No node publishes to `/{ns}/imu/data`. VINS-Fusion will receive no IMU measurements; VIO initialization is impossible.
**Fix:** Change remapping to `f'/{ns}/mavros/imu/data'`.

---

**BUG-003 | CRITICAL | `launch/fleet_sim.launch.py`, line 259**
**Title:** EuRoC calibration applied to Iris sensor suite — systematic measurement model error
`euroc_stereo_imu_config.yaml` is calibrated for the ETH Zurich EuRoC ASL camera rig (MT9V034, ADIS16448, 110mm baseline), not the Gazebo Iris sensor suite. Systematic reprojection errors in the Ceres visual residuals produce biased, rapidly-drifting state estimates even if topic routing were corrected.
**Fix:** Create `iris_gazebo_stereo_config.yaml` calibrated to actual Gazebo Iris sensor parameters using Kalibr.

---

**BUG-004 | HIGH | `launch/fleet_sim.launch.py`, lines 179–196 and 200–214**
**Title:** Dual `map → odom` TF publishers — Nav2 AMCL and slam_toolbox race condition
Both Nav2 (AMCL) and `slam_toolbox` publish the `map → odom` TF transform simultaneously. This creates a TF race condition causing `tf2::LookupException` or extrapolation errors in Nav2's costmap, planner, and controller. Navigation produces unpredictable trajectories.
**Fix:** Disable AMCL; configure Nav2 to use exclusively the `slam_toolbox`-provided TF.

---

**BUG-005 | HIGH | `launch/fleet_sim.launch.py`, lines 188–189**
**Title:** Nav2 loaded with static pre-built map while SLAM builds live map — AMCL localizer fails in new areas
As the robot explores areas not in `map.yaml`, AMCL's particle filter degenerates on unmatched scan returns. Localization fails silently; the robot uses an invalid pose estimate for planning.
**Fix:** Remove `map.yaml` argument; configure Nav2 to use the live SLAM map, or disable `slam_toolbox` and use only AMCL with the static map.

---

**BUG-006 | HIGH | `scripts/vins_swarm_fusion_node.py`, line 115**
**Title:** Keyframe orientation (quaternion) silently discarded — SE(3) reduced to R^3
`_kf_callback` stores only `(p.x, p.y, p.z, t)`. `msg.pose.orientation` is never read. Inter-drone relative pose estimation and correct 3D map construction are impossible. Published `PoseArray` always contains identity quaternions.
**Fix:** Store full 7-DOF pose `(x, y, z, qx, qy, qz, qw, t)`.

---

**BUG-007 | HIGH | `scripts/vins_swarm_fusion_node.py`, line 131**
**Title:** `loop_closure_threshold` ROS parameter ignored — hardcoded constant overrides it
`self.lc_threshold` (0.5m, from ROS parameter) is declared and read but never used. `LC_DIST_THRESH = 1.0` (hardcoded) is used instead. Parameter override in the launch file has zero runtime effect.
**Fix:** Replace `LC_DIST_THRESH = 1.0` with `self.lc_threshold`.

---

**BUG-008 | HIGH | `scripts/vins_swarm_fusion_node.py`, lines 129–145 and 157–159**
**Title:** Loop closure detection produces no pose correction — logged as "detected" but discarded
`_detect_loop_closures` returns closure candidates. `_fuse_and_publish` only logs the count. No pose-graph correction, no keyframe position update. "Loop closure detected" in logs is a false claim.
**Fix:** Integrate g2o or GTSAM pose-graph optimizer; apply SE(3) corrections upon closure detection. Or remove the misleading log until a real implementation exists.

---

**BUG-009 | HIGH | `scripts/vins_swarm_fusion_node.py` — entire fusion pipeline**
**Title:** No coordinate frame resolution between independently-initialized VIO instances
VINS-Fusion instances initialize their VIO world frames independently at an unknown relative SE(3) transformation. Concatenating keyframes into a single `map`-framed PointCloud2 produces a geometrically incorrect global map.
**Fix:** Implement inter-drone frame registration via UWB ranging or visual inter-robot loop closure with RANSAC + essential matrix decomposition.

---

**BUG-010 | MEDIUM | `config/fastdds_profile.xml`, lines 51–71**
**Title:** Named DDS profiles not applied to ROS2 topics — QoS tuning has no effect
Profiles `video_writer_profile` and `cmd_writer_profile` require explicit entity-level mapping to take effect. ROS2 with `rmw_fastrtps_cpp` only auto-applies profiles named `"default_publisher"` and `"default_subscriber"`. The video BEST_EFFORT and command RELIABLE tuning defined here is never applied to any ROS2 topic.
**Fix:** Rename profiles to `"default_publisher"` / `"default_subscriber"` or implement explicit FastDDS entity profile binding.

---

**BUG-011 | MEDIUM | `launch/fleet_sim.launch.py`, lines 302–318**
**Title:** `map_merge` init poses not passed as node parameters — `known_init_poses=True` ineffective
`multirobot_map_merge` requires `robot1_init_pose_x`, `robot1_init_pose_y`, `robot1_init_pose_yaw` (and robot2 equivalents) as ROS node parameters when `known_init_poses=True`. These are not provided. `map_merge` falls back to unknown-pose image alignment, which may fail for maps with insufficient overlap.
**Fix:** Add spawn coordinate parameters for each robot to the `map_merge_2d` node parameters dict.

---

**BUG-012 | MEDIUM | `config/zenoh_bridge.json5` — OccupancyGrid bandwidth**
**Title:** `/global_map_2d` routed over WAN at unsustainable bandwidth
100m × 100m map at 0.05m/cell = 4,000,000 cells ≈ 4 MB/message at 0.5 Hz = ~2 MB/s sustained WAN uplink. This saturates typical cloud VM bandwidth allocations.
**Fix:** Implement map delta compression, reduce WAN resolution, or serve PNG map tiles via HTTP.

---

**BUG-013 | LOW | `scripts/vins_swarm_fusion_node.py`, lines 133–144**
**Title:** O(N×M) brute-force loop closure search — quadratic scaling
500×500 = 250,000 Euclidean distance computations at 2 Hz. Does not scale with increased keyframe count.
**Fix:** Replace with `scipy.spatial.cKDTree` for O(N log M) nearest-neighbor queries.

---

**BUG-014 | LOW | `docker/docker-compose.edge.yml`, line 52**
**Title:** `ros2 topic echo --once` blocks container startup if fleet is not yet running
The `ros2_core` startup command includes `ros2 topic echo /edge/tb3_1/camera/compressed --once`, which blocks indefinitely if the fleet simulation is not publishing. This hangs the container and prevents the entire Docker stack from becoming healthy.
**Fix:** Remove the `--once` echo from the startup command; rely solely on the healthcheck.

---

## 10. Ball Aerospace-Grade Upgrade Roadmap

### Phase 1 — Demo-Readiness (Estimated: 10–12 engineering hours)

| Item | Action | Effort |
|------|--------|--------|
| P1-1 | Fix IMU remapping: `/{ns}/imu/data` → `/{ns}/mavros/imu/data` | 0.5 hr |
| P1-2 | Switch VINS-Fusion to monocular mode (`num_of_cam: 1`) | 4 hr |
| P1-3 | Create `iris_gazebo_mono_config.yaml` with Gazebo-accurate intrinsics | 2 hr |
| P1-4 | Disable AMCL in Nav2; configure to use `slam_toolbox` TF only | 2 hr |
| P1-5 | Fix `LC_DIST_THRESH` to use `self.lc_threshold` | 0.25 hr |
| P1-6 | Add init pose parameters to `map_merge` node | 0.5 hr |
| P1-7 | Remove blocking `ros2 topic echo --once` from Docker startup | 0.25 hr |
| P1-8 | Validate DDS profile application; test with `ros2 topic info --verbose` | 1 hr |

**Expected outcome:** Ground robots navigate with correct SLAM/Nav2 integration. Drones fly with working monocular VIO. Edge broker starts cleanly. Investor demo viable.

### Phase 2 — Navigation Integrity (Estimated: 4–8 engineering weeks)

| Item | Action | Effort |
|------|--------|--------|
| P2-1 | Add stereo camera pair to Iris SDF (10cm baseline) | 1 week |
| P2-2 | Calibrate stereo-IMU extrinsics with Kalibr; generate `iris_gazebo_stereo_config.yaml` | 3 days |
| P2-3 | Re-enable VINS-Fusion stereo mode with calibrated config | 1 day |
| P2-4 | Implement SE(3) pose storage in swarm fusion; resolve inter-drone frame registration | 3 days |
| P2-5 | Integrate g2o or GTSAM into swarm fusion for real pose-graph optimization | 2 weeks |
| P2-6 | Replace O(N×M) search with cKDTree; add minimum time separation constraint | 1 day |
| P2-7 | Implement OccupancyGrid delta compression for Zenoh WAN routing | 3 days |
| P2-8 | Fix FastDDS `default_publisher`/`default_subscriber` profile naming | 1 day |
| P2-9 | Override `max_laser_range` in slam_toolbox config to match LDS-01 sensor range | 0.5 day |

**Expected outcome:** Full stereo VIO operational. Pose-graph fusion produces metrically consistent 3D map. WAN routing is bandwidth-sustainable.

### Phase 3 — Production Navigation Architecture (Estimated: 3–6 months)

| Item | Action |
|------|--------|
| P3-1 | Replace VINS-Fusion with ORB-SLAM3 (stereo-inertial mode) for tracking robustness |
| P3-2 | Deploy Swarm-SLAM or COVINS-G for multi-drone map fusion with full pose-graph optimization |
| P3-3 | Integrate UWB ranging simulation for metric scale anchoring and inter-robot relative pose estimation |
| P3-4 | Replace `explore_lite` with Nav2 Coverage Planner for structured area coverage |
| P3-5 | Implement RTAB-Map multi-session mapping for ground robots with cross-session loop closure |
| P3-6 | Migrate to Zenoh Pico for embedded agents; deploy Kubernetes operator for multi-machine scaling |
| P3-7 | Integrate Prometheus alerting rules for VIO health (keyframe rate, reprojection error thresholds) |
| P3-8 | Hardware-in-the-loop validation: Intel RealSense D435i on physical drone; re-run Kalibr calibration |

---

## Summary of Findings

| Severity | Count | Primary Domain |
|----------|-------|----------------|
| CRITICAL | 3 | Drone VIO completely non-functional |
| HIGH | 6 | Ground navigation conflicts; fusion node algorithmically invalid |
| MEDIUM | 3 | Silent behavioral discrepancies; WAN bandwidth |
| LOW | 2 | Scalability and startup reliability |

The project demonstrates sound architectural thinking. The selection of VINS-Fusion, `slam_toolbox`, and the DDS/Zenoh/WebRTC pipeline are technically defensible choices used in serious robotics deployments. The gap is in the implementation details — sensor topology, topic routing, calibration, and algorithm correctness — where the current code will fail observably and immediately in a live demonstration. With Phase 1 corrections applied, the system becomes demo-viable. With Phase 2 complete, it becomes technically credible for a production pilot. Phase 3 defines a commercially competitive autonomy platform.

---

*This report constitutes a point-in-time technical assessment based on review of all provided source files. It does not constitute a certification or endorsement of the system for any operational deployment.*

**Dr. Eleanor Vasquez**
Distinguished Scientist, Navigation Systems
Ball Aerospace & Technologies Corp.
Recipient, Ball Aerospace Technical Achievement Award | Recipient, AIAA GNC Award
*February 21, 2026*
