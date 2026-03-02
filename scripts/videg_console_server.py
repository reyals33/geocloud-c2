#!/usr/bin/env python3
"""
videg_console_server.py  v1.0  (ROS2 Jazzy)
════════════════════════════════════════════════════════════
GeoCloud Fleet Operator Console — WebSocket bridge

  ROS2 /tb3_2/camera/image_raw  →  JPEG binary  →  WebSocket → Browser
  Browser JSON { linear_x, angular_z }  →  WebSocket  →  /tb3_2/cmd_vel

Usage:
  ./scripts/run_console.sh
  # or manually:
  source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
  python3 scripts/videg_console_server.py

Then open:  web/console.html  in any browser (Chromium, Firefox)
"""

import argparse
import asyncio
import functools
import http.server
import json
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False
    print('[console] WARNING: cv2 not found — install opencv-python-headless', flush=True)

try:
    import websockets
    import websockets.server
except ImportError:
    print('[console] ERROR: websockets not installed.', flush=True)
    print('  Run:  pip install "websockets>=12.0"', flush=True)
    sys.exit(1)

# ── Config ──────────────────────────────────────────────────────────────────
WS_HOST       = '0.0.0.0'
WS_PORT       = 8765
HTTP_PORT     = 8080     # serves web/console.html — open http://localhost:8080

CAMERA_TOPIC  = '/tb3_2/camera/image_raw'
CMDVEL_TOPIC  = '/tb3_2/cmd_vel'
ODOM_TOPIC    = '/tb3_2/odom'

# RTT detection thresholds: ignore simulation noise below these values
RTT_LIN_THRESHOLD = 0.02   # m/s — odom linear.x must exceed this
RTT_ANG_THRESHOLD = 0.05   # rad/s — odom angular.z must exceed this

JPEG_QUALITY  = 75       # 0-100; lower = smaller frames, faster
TARGET_FPS    = 30
FRAME_INTERVAL = 1.0 / TARGET_FPS

# Waffle camera is 1920×1080 — resize to this width before JPEG encode.
# Keeps WebSocket frames small and latency low. Set to None to disable.
STREAM_WIDTH  = 1280     # px; height auto-scales to maintain aspect ratio

# TurtleBot3 Waffle safe speed limits
LINEAR_MAX    =  0.26    # m/s
LINEAR_MIN    = -0.26    # m/s
ANGULAR_MAX   =  1.82    # rad/s
ANGULAR_MIN   = -1.82    # rad/s


# ── Image decode (no cv_bridge dependency) ───────────────────────────────────
_ENCODING_CHANNELS = {
    'rgb8':  ('rgb',  3),
    'bgr8':  ('bgr',  3),
    'rgba8': ('rgba', 4),
    'bgra8': ('bgra', 4),
    'mono8': ('mono', 1),
}

def _ros_image_to_jpeg(msg: Image) -> bytes | None:
    """Convert a ROS2 Image message to JPEG bytes using numpy + cv2."""
    if not _HAS_CV2:
        return None

    enc = msg.encoding.lower()
    if enc not in _ENCODING_CHANNELS:
        return None

    fmt, channels = _ENCODING_CHANNELS[enc]
    try:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels
        )
        # cv2.imencode expects BGR; convert if needed
        if fmt == 'rgb':
            arr = arr[:, :, ::-1]              # RGB → BGR
        elif fmt == 'rgba':
            arr = arr[:, :, 2::-1]             # RGBA → BGR
        elif fmt == 'bgra':
            arr = arr[:, :, :3]                # BGRA → BGR
        elif fmt == 'mono':
            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

        # Resize if the frame is wider than STREAM_WIDTH (e.g. 1920→1280)
        if STREAM_WIDTH and arr.shape[1] > STREAM_WIDTH:
            scale = STREAM_WIDTH / arr.shape[1]
            new_h = int(arr.shape[0] * scale)
            arr = cv2.resize(arr, (STREAM_WIDTH, new_h), interpolation=cv2.INTER_LINEAR)

        ok, buf = cv2.imencode(
            '.jpg', arr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        )
        return bytes(buf) if ok else None
    except Exception as exc:
        return None


# ── ROS2 Node ────────────────────────────────────────────────────────────────
class ConsoleNode(Node):
    def __init__(self):
        super().__init__('videg_console_server')

        self._latest_jpeg: bytes | None = None
        self._frame_lock  = threading.Lock()
        self._last_frame_t = 0.0
        self._frame_count  = 0

        # RTT tracking: measure time from non-zero cmd_vel → odom velocity response
        self._rtt_lock       = threading.Lock()
        self._pending_cmd_t  = 0.0          # monotonic timestamp of last zero→nonzero cmd
        self._last_cmd_zero  = True         # True when last published cmd was zero
        self._cmd_rtt_ms: float | None = None  # most recent measured RTT
        self._odom_lx        = 0.0
        self._odom_az        = 0.0

        self._sub_cam = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self._image_cb,
            qos_profile_sensor_data,
        )
        self._sub_odom = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_cb,
            qos_profile_sensor_data,
        )
        self._pub_cmd = self.create_publisher(
            Twist,
            CMDVEL_TOPIC,
            10,
        )

        self.get_logger().info(
            f'ConsoleNode up | {CAMERA_TOPIC} → WS:{WS_PORT} | '
            f'WS cmd → {CMDVEL_TOPIC} | odom RTT ← {ODOM_TOPIC}'
        )

    # ── Camera callback ──────────────────────────────────────────────────────
    def _image_cb(self, msg: Image) -> None:
        jpeg = _ros_image_to_jpeg(msg)
        if jpeg:
            with self._frame_lock:
                self._latest_jpeg  = jpeg
                self._last_frame_t = time.monotonic()
                self._frame_count += 1
        elif self._frame_count == 0:
            self.get_logger().warn(
                f'Cannot encode image (encoding={msg.encoding}, '
                f'cv2={_HAS_CV2}) — no video will be streamed.',
                throttle_duration_sec=10.0,
            )

    def get_latest_jpeg(self) -> bytes | None:
        with self._frame_lock:
            return self._latest_jpeg

    def frame_age(self) -> float:
        """Seconds since the last camera frame arrived."""
        with self._frame_lock:
            if self._last_frame_t == 0.0:
                return float('inf')
            return time.monotonic() - self._last_frame_t

    def frame_count(self) -> int:
        with self._frame_lock:
            return self._frame_count

    # ── Odometry callback — measures cmd→motion RTT ──────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        lx = msg.twist.twist.linear.x
        az = msg.twist.twist.angular.z
        with self._rtt_lock:
            self._odom_lx = lx
            self._odom_az = az
            if self._pending_cmd_t > 0.0:
                if abs(lx) > RTT_LIN_THRESHOLD or abs(az) > RTT_ANG_THRESHOLD:
                    rtt = (time.monotonic() - self._pending_cmd_t) * 1000.0
                    self._cmd_rtt_ms   = rtt
                    self._pending_cmd_t = 0.0

    def get_telemetry(self) -> dict:
        with self._rtt_lock:
            return {
                'type':       'telemetry',
                'cmd_rtt_ms': round(self._cmd_rtt_ms, 1) if self._cmd_rtt_ms is not None else None,
                'odom_lx':    round(self._odom_lx, 3),
                'odom_az':    round(self._odom_az, 3),
            }

    # ── cmd_vel publish ──────────────────────────────────────────────────────
    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x  = max(LINEAR_MIN, min(LINEAR_MAX, linear_x))
        msg.angular.z = max(ANGULAR_MIN, min(ANGULAR_MAX, angular_z))
        self._pub_cmd.publish(msg)

        # RTT: record timestamp on zero → nonzero transition
        nonzero = abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001
        with self._rtt_lock:
            if nonzero and self._last_cmd_zero:
                self._pending_cmd_t = time.monotonic()
            elif not nonzero:
                self._pending_cmd_t = 0.0   # cancelled; discard pending measurement
            self._last_cmd_zero = not nonzero

    def stop(self) -> None:
        self.publish_cmd(0.0, 0.0)


# ── Module-level node ref (set in main before serve()) ───────────────────────
_node: ConsoleNode | None = None


# ── WebSocket handler ────────────────────────────────────────────────────────
async def ws_handler(websocket) -> None:
    remote = websocket.remote_address
    _node.get_logger().info(f'Operator connected: {remote}')

    async def send_video() -> None:
        """Push JPEG frames and periodic telemetry JSON to the client."""
        last_jpeg: bytes | None = None
        last_telem_t = 0.0
        while True:
            t0 = time.monotonic()
            jpeg = _node.get_latest_jpeg()

            if jpeg is not None and jpeg is not last_jpeg:
                try:
                    await websocket.send(jpeg)
                    last_jpeg = jpeg
                except websockets.exceptions.ConnectionClosed:
                    return

            # Push telemetry JSON every 500 ms (interleaved with JPEG frames)
            if t0 - last_telem_t >= 0.5:
                try:
                    await websocket.send(json.dumps(_node.get_telemetry()))
                    last_telem_t = t0
                except websockets.exceptions.ConnectionClosed:
                    return

            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.0, FRAME_INTERVAL - elapsed))

    async def recv_cmds() -> None:
        """Receive JSON cmd_vel commands from the operator console."""
        async for raw in websocket:
            if not isinstance(raw, str):
                continue
            try:
                data = json.loads(raw)
                _node.publish_cmd(
                    float(data.get('linear_x',  0.0)),
                    float(data.get('angular_z', 0.0)),
                )
            except Exception as exc:
                _node.get_logger().warn(
                    f'Bad command from {remote}: {exc}',
                    throttle_duration_sec=2.0,
                )

    try:
        await asyncio.gather(send_video(), recv_cmds())
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        _node.get_logger().info(f'Operator disconnected: {remote}')
        _node.stop()   # safety stop on disconnect


# ── Diagnostics loop ─────────────────────────────────────────────────────────
async def diagnostics_loop() -> None:
    """Log camera status every 10 s so the user knows if frames are arriving."""
    last_count = 0
    while True:
        await asyncio.sleep(10.0)
        count = _node.frame_count()
        age   = _node.frame_age()
        delta = count - last_count
        last_count = count

        if delta == 0:
            _node.get_logger().warn(
                f'[console] No camera frames in last 10 s. '
                f'Check that {CAMERA_TOPIC} is publishing. '
                f'(Verify GZ camera topic: gz topic -l | grep camera)'
            )
        else:
            _node.get_logger().info(
                f'[console] Camera OK — {delta} frames/10s, '
                f'last frame {age:.2f}s ago'
            )


# ── Main ─────────────────────────────────────────────────────────────────────
def _start_http_server(web_dir: str) -> None:
    """Serve the web/ directory over HTTP in a background daemon thread.
    Lets Windows browsers (WSL2) open the console via http://localhost:8080."""
    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler,
        directory=web_dir,
    )
    # suppress per-request log lines to keep console output clean
    handler.log_message = lambda *_: None
    httpd = http.server.HTTPServer(('0.0.0.0', HTTP_PORT), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True, name='http')
    thread.start()


def main() -> None:
    global _node

    # Parse our args first; pass remaining to rclpy (handles --ros-args etc.)
    parser = argparse.ArgumentParser(
        description='ViDeG operator console — WebSocket bridge',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--cloud', metavar='WS_URL', default=None,
        help=(
            'Connect OUT to a cloud relay instead of listening for browsers. '
            'Example: --cloud ws://EC2_IP:8766  '
            'Use videg_relay.py on EC2 (or localhost for pre-flight testing).'
        ),
    )
    our_args, ros_argv = parser.parse_known_args()

    rclpy.init(args=ros_argv)
    _node = ConsoleNode()

    # Spin ROS2 in a background daemon thread
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(_node,),
        daemon=True,
        name='ros_spin',
    )
    spin_thread.start()

    repo_root = __file__.rsplit('/scripts/', 1)[0]
    web_dir   = f'{repo_root}/web'

    # HTTP server so Windows/WSL2 browsers can open the console UI
    # (still useful in --cloud mode for local pre-flight: browser on port 8080,
    #  WS URL changed to point at relay port 8765)
    _start_http_server(web_dir)

    cloud_url = our_args.cloud

    if cloud_url:
        print(f'\n{"═"*58}')
        print(f'  GeoCloud Fleet Console  —  CLOUD MODE  v1.1')
        print(f'{"═"*58}')
        print(f'  Relay:       {cloud_url}')
        print(f'  Local UI:    http://localhost:{HTTP_PORT}/console.html')
        print(f'  Camera:      {CAMERA_TOPIC}')
        print(f'  cmd_vel:     {CMDVEL_TOPIC}')
        print(f'{"═"*58}')
        print(f'  Edge is connecting OUT to relay (no open inbound ports needed).')
        print(f'  Browser connects to relay browser port (e.g. ws://RELAY:8765).')
        print(f'{"═"*58}\n', flush=True)
    else:
        print(f'\n{"═"*58}')
        print(f'  GeoCloud Fleet Console  —  LOCAL MODE  v1.1')
        print(f'{"═"*58}')
        print(f'  Console UI:  http://localhost:{HTTP_PORT}/console.html')
        print(f'  WebSocket:   ws://localhost:{WS_PORT}')
        print(f'  Camera:      {CAMERA_TOPIC}')
        print(f'  cmd_vel:     {CMDVEL_TOPIC}')
        print(f'{"═"*58}')
        print(f'  Open the Console UI URL above in your browser.')
        print(f'  Drive with WASD / arrow keys or the on-screen D-pad.')
        print(f'{"═"*58}\n', flush=True)

    async def serve_local() -> None:
        """Local mode: listen for browser connections on WS_PORT."""
        async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
            await asyncio.gather(
                asyncio.Future(),      # run forever
                diagnostics_loop(),
            )

    async def serve_cloud(relay_url: str) -> None:
        """Cloud mode: connect OUT to relay; reconnect automatically on drop."""
        RETRY_DELAY = 5.0

        async def _connect_loop() -> None:
            while True:
                try:
                    _node.get_logger().info(f'[cloud] Connecting to relay: {relay_url}')
                    async with websockets.connect(relay_url) as ws:
                        _node.get_logger().info('[cloud] Relay connected — streaming.')
                        await ws_handler(ws)
                except (websockets.exceptions.ConnectionClosed, OSError) as exc:
                    _node.get_logger().warn(
                        f'[cloud] Relay connection lost ({exc}) — '
                        f'retrying in {RETRY_DELAY:.0f}s'
                    )
                    await asyncio.sleep(RETRY_DELAY)

        await asyncio.gather(_connect_loop(), diagnostics_loop())

    try:
        if cloud_url:
            asyncio.run(serve_cloud(cloud_url))
        else:
            asyncio.run(serve_local())
    except KeyboardInterrupt:
        print('\n[console] Shutting down — sending stop command...')
    finally:
        if _node:
            _node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
