#!/usr/bin/env bash
# ros2_to_rtsp.sh  v2.1.0
# ─────────────────────────────────────────────────────────────────────
# Entrypoint: launches ros2_to_rtsp_node.py which subscribes to
#   /<ns>/camera/compressed  (CompressedImage, BEST_EFFORT QoS)
# and pipes raw H.264 bytes directly into FFmpeg → RTSP (MediaMTX).
#
# Usage:  ros2_to_rtsp.sh <robot_namespace> <rtsp_output_url>
# Example: ros2_to_rtsp.sh drone_1 rtsp://mediamtx:8554/drone_1
#
# Change log:
#   v1.0.0  Original bash/YAML-parse approach (broken: heredoc variable
#           expansion suppressed by single-quoted <<'PYEOF'; integer
#           parsing of YAML uint8 arrays was O(N) per frame byte-by-byte)
#   v2.1.0  Replaced with ros2_to_rtsp_node.py — proper rclpy subscriber
#           that receives CompressedImage.data as binary bytes and writes
#           directly to FFmpeg stdin. Eliminates YAML parsing overhead,
#           variable-expansion bug, and ~80 ms serialization latency.
# ─────────────────────────────────────────────────────────────────────
set -euo pipefail

ROBOT_NS="${1:?Usage: ros2_to_rtsp.sh <ns> <rtsp_url>}"
RTSP_OUT="${2:?Usage: ros2_to_rtsp.sh <ns> <rtsp_url>}"

echo "[ros2_to_rtsp] Starting relay: /${ROBOT_NS}/camera/compressed → ${RTSP_OUT}"

# Source ROS2 if not already sourced
if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "${SCRIPT_DIR}/ros2_to_rtsp_node.py" \
  "${ROBOT_NS}" "${RTSP_OUT}"
