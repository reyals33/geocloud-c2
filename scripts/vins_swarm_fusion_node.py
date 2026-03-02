#!/usr/bin/env -S /home/georg/projects/ViDeG/.venv/bin/python3
"""
vins_swarm_fusion_node.py
────────────────────────────────────────────────────────────────────────
Multi-drone VINS pose-graph fusion node. v2.1.0 (post-expert-review)

Fixes applied (2026-02-21):
  - SE(3) pose storage: quaternion (qx,qy,qz,qw) preserved alongside position
  - loop_closure_threshold ROS parameter now actually used (was hardcoded 1.0m)
  - O(N²) brute-force nearest-neighbor replaced with scipy.spatial.cKDTree O(N log N)
  - frame_id changed from 'map' to 'vins_world' to avoid TF conflict with slam_toolbox
  - PoseArray now publishes real stored orientation instead of identity quaternion

For production-scale fusion, replace with:
  - Omni-swarm (decentralized VIO + UWB/visual inter-robot ranging)
  - COVINS-G    (centralized keyframe graph fusion, g2o backend)
  - Swarm-SLAM  (decentralized, sparse, communication-efficient)

ROS2 Topics:
  Subscribed:
    /drone_1/vins/keyframe_pose  (geometry_msgs/PoseStamped)
    /drone_2/vins/keyframe_pose  (geometry_msgs/PoseStamped)
  Published:
    /global_map_3d               (sensor_msgs/PointCloud2)   frame: vins_world
    /swarm/drone_poses           (geometry_msgs/PoseArray)   frame: vins_world
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Pose, Point, PoseStamped, PoseArray, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import numpy as np
import struct
from collections import defaultdict

try:
    from scipy.spatial import cKDTree
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False


class SwarmFusionNode(Node):

    def __init__(self):
        super().__init__('vins_swarm_fusion')

        # ── Parameters ─────────────────────────────────────────
        self.declare_parameter('num_drones', 2)
        self.declare_parameter('fuse_rate', 2.0)             # Hz
        self.declare_parameter('loop_closure_threshold', 0.5)
        self.declare_parameter('max_keyframes_per_drone', 500)

        self.num_drones   = self.get_parameter('num_drones').value
        self.fuse_rate    = self.get_parameter('fuse_rate').value
        self.lc_threshold = self.get_parameter('loop_closure_threshold').value
        self.max_kf       = self.get_parameter('max_keyframes_per_drone').value

        if not _SCIPY_AVAILABLE:
            self.get_logger().warn(
                'scipy not available — loop closure uses slower fallback. '
                'Install with: pip install scipy'
            )

        # ── State: keyframe pose history per drone ──────────────
        # Each entry: (x, y, z, qx, qy, qz, qw, timestamp)
        self.keyframes: dict[str, list[tuple]] = defaultdict(list)

        # ── QoS ────────────────────────────────────────────────
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_transient = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Subscribers ─────────────────────────────────────────
        drone_ns_list = [f'drone_{i+1}' for i in range(self.num_drones)]
        for ns in drone_ns_list:
            topic = f'/{ns}/vins/keyframe_pose'
            self.create_subscription(
                PoseStamped, topic,
                lambda msg, _ns=ns: self._kf_callback(msg, _ns),
                qos_reliable,
            )
            self.get_logger().info(f'Subscribed: {topic}')

        # ── Publishers ──────────────────────────────────────────
        # frame_id = 'vins_world' to avoid conflict with slam_toolbox 'map' frame.
        # A static transform from vins_world → map is published by fleet_sim.launch.py
        # using drone spawn coordinates as the transform origin.
        self.pub_cloud = self.create_publisher(
            PointCloud2, '/global_map_3d', qos_transient
        )
        self.pub_poses = self.create_publisher(
            PoseArray, '/swarm/drone_poses', qos_best_effort
        )

        # ── Fusion timer ────────────────────────────────────────
        self.create_timer(1.0 / self.fuse_rate, self._fuse_and_publish)

        self.get_logger().info(
            f'SwarmFusionNode ready. num_drones={self.num_drones}, '
            f'fuse_rate={self.fuse_rate} Hz, '
            f'lc_threshold={self.lc_threshold} m'
        )

    def _kf_callback(self, msg: PoseStamped, ns: str):
        """Store incoming keyframe SE(3) pose; limit buffer size."""
        p = msg.pose.position
        q = msg.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Store full SE(3): position (x,y,z) + orientation quaternion (qx,qy,qz,qw)
        self.keyframes[ns].append((p.x, p.y, p.z, q.x, q.y, q.z, q.w, t))

        # Trim oldest frames if buffer exceeds limit
        if len(self.keyframes[ns]) > self.max_kf:
            self.keyframes[ns] = self.keyframes[ns][-self.max_kf:]

    def _detect_loop_closures(self):
        """
        Spatial loop closure heuristic using cKDTree for O(N log N) performance.

        Uses self.lc_threshold (from ROS parameter 'loop_closure_threshold').
        NOTE: Spatial proximity alone is not a valid loop closure criterion for VIO.
        Production systems must use descriptor matching (DBoW3, NetVLAD) for
        appearance-based verification before accepting a closure.

        Returns: list of (ns_a, idx_a, ns_b, idx_b) candidate closure pairs
        """
        closures = []
        drones = list(self.keyframes.keys())

        for i, ns_a in enumerate(drones):
            for ns_b in drones[i+1:]:
                kf_a = self.keyframes[ns_a]
                kf_b = self.keyframes[ns_b]
                if not kf_a or not kf_b:
                    continue

                pts_a = np.array([[k[0], k[1], k[2]] for k in kf_a])
                pts_b = np.array([[k[0], k[1], k[2]] for k in kf_b])

                if _SCIPY_AVAILABLE:
                    # O(N log N) kd-tree query
                    tree_b = cKDTree(pts_b)
                    dists, idxs = tree_b.query(pts_a, k=1,
                                               distance_upper_bound=self.lc_threshold)
                    for ia, (dist, ib) in enumerate(zip(dists, idxs)):
                        if dist < self.lc_threshold:
                            closures.append((ns_a, ia, ns_b, int(ib)))
                else:
                    # Fallback: vectorized numpy (no external deps)
                    for ia, pa in enumerate(pts_a):
                        dists = np.linalg.norm(pts_b - pa, axis=1)
                        min_idx = int(np.argmin(dists))
                        if dists[min_idx] < self.lc_threshold:
                            closures.append((ns_a, ia, ns_b, min_idx))

        return closures

    def _fuse_and_publish(self):
        """Merge all keyframes into a global PointCloud2 and publish."""
        all_points = []
        for kfs in self.keyframes.values():
            all_points.extend([(k[0], k[1], k[2]) for k in kfs])

        if not all_points:
            return

        # Loop closure detection (spatial heuristic — see note in _detect_loop_closures)
        closures = self._detect_loop_closures()
        if closures:
            self.get_logger().info(
                f'Detected {len(closures)} potential loop closure candidate(s) '
                f'(spatial heuristic only — no pose correction applied)'
            )

        # Build PointCloud2 in vins_world frame
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'vins_world'   # distinct from slam_toolbox 'map' frame

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        data = b''.join(struct.pack('fff', *p) for p in all_points)

        cloud = PointCloud2(
            header=header,
            height=1,
            width=len(all_points),
            fields=fields,
            is_bigendian=False,
            point_step=point_step,
            row_step=point_step * len(all_points),
            data=data,
            is_dense=True,
        )
        self.pub_cloud.publish(cloud)

        # Publish current drone pose array with real stored orientations
        poses_msg = PoseArray()
        poses_msg.header = header
        for kfs in self.keyframes.values():
            if kfs:
                x, y, z, qx, qy, qz, qw, _ = kfs[-1]
                poses_msg.poses.append(Pose(
                    position=Point(x=x, y=y, z=z),
                    orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
                ))
        self.pub_poses.publish(poses_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
