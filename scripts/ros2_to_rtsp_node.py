#!/usr/bin/env -S /home/georg/projects/ViDeG/.venv/bin/python3
"""
ros2_to_rtsp_node.py
────────────────────────────────────────────────────────────────────────
ROS2 CompressedImage → FFmpeg → RTSP relay node. v2.1.0

Replaces the fragile ros2_to_rtsp.sh YAML-text-parsing approach with a
proper rclpy subscriber that writes binary H.264 bytes directly to FFmpeg.

Fixes from expert review:
  - Eliminates heredoc variable expansion bug (original ros2_to_rtsp.sh)
  - Binary pipe instead of YAML text parsing (correct H.264 byte stream)
  - Runs inside ros:humble-ros-base container (has ROS2; linuxserver/ffmpeg did not)
  - Frame boundary preserved — msg.data is a complete H.264 NAL unit

Usage:
  python3 ros2_to_rtsp_node.py <robot_namespace> <rtsp_output_url>

Example:
  python3 ros2_to_rtsp_node.py drone_1 rtsp://localhost:8554/drone_1

Dependencies:
  - ROS2 Humble (rclpy, sensor_msgs)
  - ffmpeg installed in the container/system
"""

import sys
import subprocess
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage


class VideoRelayNode(Node):
    """Subscribes to /{ns}/camera/compressed and pipes H.264 bytes to FFmpeg RTSP."""

    def __init__(self, ns: str, rtsp_url: str):
        super().__init__(f'video_relay_{ns}')
        self._ns = ns
        self._rtsp_url = rtsp_url
        self._ffmpeg = None
        self._frame_count = 0

        # Start FFmpeg subprocess
        self._start_ffmpeg()

        # Subscribe to compressed image topic (BEST_EFFORT, KEEP_LAST=1)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = f'/{ns}/camera/compressed'
        self.create_subscription(CompressedImage, topic, self._callback, qos)
        self.get_logger().info(f'[{ns}] Relaying {topic} → {rtsp_url}')

        # Watchdog: log throughput every 10 seconds
        self.create_timer(10.0, self._log_stats)

    def _start_ffmpeg(self):
        """Launch FFmpeg reading H.264 ES from stdin, pushing RTSP out."""
        cmd = [
            'ffmpeg',
            '-fflags',         'nobuffer',     # disable input buffering (~100ms reduction)
            '-flags',          'low_delay',    # single-frame decode mode
            '-f',              'h264',         # input format: raw H.264 elementary stream
            '-i',              'pipe:0',       # read from stdin
            '-c:v',            'copy',         # no re-encode; preserve zerolatency from encoder
            '-an',                             # no audio
            '-f',              'rtsp',
            '-rtsp_transport', 'tcp',          # reliable; use udp for LAN lower latency
            '-muxdelay',       '0',            # zero output mux delay
            self._rtsp_url,
        ]
        self._ffmpeg = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.get_logger().info(f'[{self._ns}] FFmpeg started, PID={self._ffmpeg.pid}')

    def _callback(self, msg: CompressedImage):
        """Write H.264 bytes from CompressedImage directly to FFmpeg stdin."""
        if self._ffmpeg is None or self._ffmpeg.stdin is None:
            return

        # Check if FFmpeg process is still alive
        if self._ffmpeg.poll() is not None:
            self.get_logger().warn(
                f'[{self._ns}] FFmpeg exited (code={self._ffmpeg.returncode}), restarting...'
            )
            self._start_ffmpeg()
            return

        try:
            # msg.data is bytes — H.264 NAL units, no conversion needed
            self._ffmpeg.stdin.write(bytes(msg.data))
            self._ffmpeg.stdin.flush()
            self._frame_count += 1
        except BrokenPipeError:
            self.get_logger().error(f'[{self._ns}] FFmpeg pipe broken — will restart on next frame')
            self._ffmpeg = None

    def _log_stats(self):
        """Periodic throughput log."""
        self.get_logger().info(
            f'[{self._ns}] Relayed {self._frame_count} frames total → {self._rtsp_url}'
        )

    def destroy_node(self):
        """Clean up FFmpeg subprocess on shutdown."""
        if self._ffmpeg and self._ffmpeg.poll() is None:
            self._ffmpeg.terminate()
            try:
                self._ffmpeg.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._ffmpeg.kill()
        super().destroy_node()


def main():
    if len(sys.argv) != 3:
        print('Usage: ros2_to_rtsp_node.py <robot_namespace> <rtsp_output_url>')
        print('Example: ros2_to_rtsp_node.py drone_1 rtsp://localhost:8554/drone_1')
        sys.exit(1)

    ns = sys.argv[1]
    rtsp_url = sys.argv[2]

    rclpy.init()
    node = VideoRelayNode(ns, rtsp_url)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
