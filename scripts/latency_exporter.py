#!/usr/bin/env -S /home/georg/projects/ViDeG/.venv/bin/python3
"""
latency_exporter.py
────────────────────────────────────────────────────────────────────────
Prometheus metrics exporter for ROS2 topic latency. v2.1.0

Publishes the following metrics on port 9091 for Grafana scraping:
  ros2_topic_latency_ms{namespace, topic}       – header.stamp delay in ms
  ros2_topic_hz{namespace, topic}               – message frequency (Hz)
  ros2_topic_high_latency_total{namespace,topic}– frames exceeding warn threshold

Fix (v2.1.0): Counter semantics corrected.
  Previously: DROPPED counter incremented on high latency (wrong — those frames
              ARRIVED, they were not dropped; truly dropped frames never reach callback).
  Now: ros2_topic_high_latency_total counts frames that arrived but exceeded the
       threshold. True drop detection would require sequence number tracking which
       sensor_msgs/CompressedImage does not provide.

Run:   python3 latency_exporter.py
       Then browse: http://localhost:9091/metrics

Dependencies:
  pip install prometheus-client rclpy
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from prometheus_client import start_http_server, Gauge, Counter

# ── Prometheus metrics ─────────────────────────────────────────────────
LATENCY_MS = Gauge(
    'ros2_topic_latency_ms',
    'End-to-end latency from header.stamp to receive time (ms)',
    ['namespace', 'topic']
)
TOPIC_HZ = Gauge(
    'ros2_topic_hz',
    'Message frequency for topic (Hz)',
    ['namespace', 'topic']
)
HIGH_LATENCY = Counter(
    'ros2_topic_high_latency_total',
    'Frames received above latency warning threshold (not dropped — arrived late)',
    ['namespace', 'topic']
)

# ── Fleet config ───────────────────────────────────────────────────────
FLEET_NAMESPACES = ['tb3_1', 'tb3_2', 'drone_1', 'drone_2']
MONITORED_TOPIC  = 'camera/compressed'
LATENCY_WARN_MS  = 150.0    # Log warning above this threshold


class LatencyMonitor(Node):
    """Subscribes to compressed image topics for all fleet members."""

    def __init__(self):
        super().__init__('latency_exporter')
        self._msg_times: dict[str, list[float]] = {}
        self._hz_window = 5.0   # seconds for Hz calculation

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for ns in FLEET_NAMESPACES:
            topic = f'/{ns}/{MONITORED_TOPIC}'
            self._msg_times[ns] = []

            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, _ns=ns: self._callback(msg, _ns),
                qos,
            )
            self.get_logger().info(f'Monitoring: {topic}')

        # Hz calculation timer (runs every second)
        self.create_timer(1.0, self._update_hz)

    def _callback(self, msg: CompressedImage, ns: str):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)

        if stamp_ns == 0:
            return   # No timestamp embedded – skip

        latency_ms = (now_ns - stamp_ns) / 1_000_000.0

        LATENCY_MS.labels(namespace=ns, topic=MONITORED_TOPIC).set(latency_ms)
        self._msg_times[ns].append(time.monotonic())

        if latency_ms > LATENCY_WARN_MS:
            self.get_logger().warn(
                f'[{ns}] HIGH LATENCY: {latency_ms:.1f} ms > {LATENCY_WARN_MS} ms '
                f'(frame arrived late, not dropped)'
            )
            # Increment high-latency counter (NOT a drop — frame arrived)
            HIGH_LATENCY.labels(namespace=ns, topic=MONITORED_TOPIC).inc()

    def _update_hz(self):
        cutoff = time.monotonic() - self._hz_window
        for ns in FLEET_NAMESPACES:
            self._msg_times[ns] = [t for t in self._msg_times[ns] if t > cutoff]
            hz = len(self._msg_times[ns]) / self._hz_window
            TOPIC_HZ.labels(namespace=ns, topic=MONITORED_TOPIC).set(hz)


def main():
    # Start Prometheus HTTP server
    start_http_server(9091)
    print('[latency_exporter] Prometheus metrics on :9091/metrics')

    rclpy.init()
    node = LatencyMonitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
