#!/usr/bin/env python3
"""
gst_webrtc_agent.py — GStreamer WebRTC producer for one C2 camera agent.

Subscribes to /{agent}/camera/image_raw (sensor_msgs/Image) via ROS2,
encodes with NVENC (nvh264enc) or CPU fallback (x264enc),
and streams to a browser via WebRTC using GStreamer's webrtcbin.

Architecture:
  ROS2 image_raw → appsrc → videoconvert → [nvh264enc|x264enc] →
  rtph264pay → webrtcbin ←→ signaling server ←→ browser RTCPeerConnection

Trickle ICE: offer SDP is sent immediately; ICE candidates are forwarded
individually via the signaling server as they are gathered.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 gst_webrtc_agent.py robot1           # NVENC (default)
  python3 gst_webrtc_agent.py drone2 --sw      # software x264 fallback
  python3 gst_webrtc_agent.py robot1 ws://localhost:8765  # custom signal URL

EC2 prerequisites (run once):
  sudo apt-get install -y \\
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \\
      gstreamer1.0-plugins-bad  gstreamer1.0-plugins-ugly \\
      gstreamer1.0-tools        python3-gi gir1.2-gstreamer-1.0 \\
      gir1.2-gst-plugins-bad-1.0
  # NVENC requires the NVIDIA driver (nvidia-driver-535 or newer)
  # x264enc requires gstreamer1.0-plugins-ugly (already in apt above)
"""

import sys
import asyncio
import json
import logging
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import gi
gi.require_version('Gst',       '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp',    '1.0')
from gi.repository import Gst, GstWebRTC, GstSdp, GLib  # GLib needed for MainLoop

import websockets

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s [%(name)s] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('gst_webrtc')

DEFAULT_SIGNAL_URL = 'ws://localhost:8765'
DEFAULT_WIDTH  = 320
DEFAULT_HEIGHT = 240
DEFAULT_FPS    = 30


# ── GStreamer producer ─────────────────────────────────────────────────────────

class GstWebRTCProducer:
    """
    Manages a GStreamer pipeline with webrtcbin for one agent.

    Pipeline (NVENC):
      appsrc → videoconvert → video/x-raw,I420 →
      nvh264enc preset=low-latency-hq →
      rtph264pay config-interval=1 pt=96 →
      application/x-rtp → webrtcbin

    Pipeline (software):
      appsrc → videoconvert → video/x-raw,I420 →
      x264enc tune=zerolatency speed-preset=ultrafast →
      rtph264pay config-interval=1 pt=96 →
      application/x-rtp → webrtcbin
    """

    def __init__(self, agent_id: str, signal_url: str, use_hw: bool):
        self.agent_id   = agent_id
        self.signal_url = signal_url
        self.use_hw     = use_hw

        self.pipeline:  Gst.Pipeline | None  = None
        self.appsrc:    Gst.Element  | None  = None
        self.webrtcbin: Gst.Element  | None  = None
        self.ws:        websockets.WebSocketClientProtocol | None = None
        self.loop:      asyncio.AbstractEventLoop | None = None

        # Image geometry (determined from first ROS2 frame)
        self.width  = DEFAULT_WIDTH
        self.height = DEFAULT_HEIGHT
        self.fps    = DEFAULT_FPS

        self._pipe_ready  = False
        self._pts         = 0
        self._frame_dur   = Gst.SECOND // DEFAULT_FPS
        self._encoding    = 'rgb8'  # updated on first frame

        # Protects pipeline lifecycle between the ROS2 thread (push_frame)
        # and the asyncio thread (_reset_peer). Only acquired briefly.
        self._pipeline_lock = threading.Lock()

    # ── Pipeline ──────────────────────────────────────────────────────────────

    def _build_pipeline(self, w: int, h: int, fps: int, encoding: str):
        self.width, self.height, self.fps = w, h, fps
        self._frame_dur = Gst.SECOND // fps
        self._encoding  = encoding

        if self.use_hw:
            enc = (
                'nvh264enc preset=low-latency-hq bitrate=3000 zerolatency=true'
            )
        else:
            enc = (
                'x264enc tune=zerolatency bitrate=1500 speed-preset=ultrafast'
            )

        pipe_str = (
            f'appsrc name=appsrc is-live=true format=time '
            f'  caps=video/x-raw,format=RGB,width={w},height={h},framerate={fps}/1 ! '
            f'videoconvert ! '
            f'video/x-raw,format=I420 ! '
            f'{enc} ! '
            f'rtph264pay config-interval=1 pt=96 ! '
            f'application/x-rtp,media=video,payload=96,encoding-name=H264 ! '
            f'webrtcbin name=webrtc bundle-policy=max-bundle '
            f'  stun-server=stun://stun.l.google.com:19302'
        )

        log.info('[%s] building pipeline: %s', self.agent_id, pipe_str)
        self.pipeline  = Gst.parse_launch(pipe_str)
        self.appsrc    = self.pipeline.get_by_name('appsrc')
        self.webrtcbin = self.pipeline.get_by_name('webrtc')

        self.webrtcbin.connect('on-negotiation-needed', self._on_negotiation_needed)
        self.webrtcbin.connect('on-ice-candidate',      self._on_ice_candidate)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::error', self._on_bus_error)
        bus.connect('message::eos',   self._on_bus_eos)

        self.pipeline.set_state(Gst.State.PLAYING)
        self._pipe_ready = True
        log.info('[%s] pipeline PLAYING (encoder=%s, %dx%d @ %dfps)',
                 self.agent_id,
                 'nvh264enc' if self.use_hw else 'x264enc',
                 w, h, fps)

    def push_frame(self, msg: Image):
        """Push a ROS2 Image into appsrc. Called from the ROS2 thread.

        Thread-safety: _reset_peer (asyncio thread) may set self.appsrc = None
        concurrently. We snapshot the reference before use; if the pipeline
        is being torn down, emit returns FLUSHING (non-crash, just a log).
        """
        # Fast check outside lock; double-check inside to avoid concurrent builds.
        if not self._pipe_ready:
            with self._pipeline_lock:
                if not self._pipe_ready:
                    self._build_pipeline(msg.width, msg.height, self.fps, msg.encoding)

        # Atomic snapshot — safe even if _reset_peer fires immediately after.
        appsrc = self.appsrc
        if appsrc is None:
            return

        # Gazebo publishes rgb8; flip to RGB if bgr8
        if msg.encoding in ('bgr8', 'BGR8'):
            import numpy as np
            arr  = np.frombuffer(msg.data, dtype='uint8').reshape(msg.height, msg.width, 3)
            data = arr[:, :, ::-1].tobytes()
        else:
            data = bytes(msg.data)

        buf          = Gst.Buffer.new_wrapped(data)
        buf.pts      = self._pts
        buf.dts      = self._pts
        buf.duration = self._frame_dur
        self._pts   += self._frame_dur

        ret = appsrc.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            log.warning('[%s] push-buffer returned: %s', self.agent_id, ret)

    def _on_negotiation_needed(self, webrtcbin):
        """Fired by webrtcbin when a peer has been added and SDP is needed."""
        log.info('[%s] negotiation needed — creating offer', self.agent_id)
        promise = Gst.Promise.new_with_change_func(
            self._on_offer_created, None, None
        )
        webrtcbin.emit('create-offer', None, promise)

    def _on_offer_created(self, promise, _u, _v):
        """Callback when offer SDP is ready — set local description and send immediately.

        Trickle ICE: the offer SDP does not contain candidates. Candidates
        arrive separately via _on_ice_candidate and are forwarded to the browser
        as they are gathered. No blocking wait here — the GLib callback must
        return quickly to avoid starving the GLib main loop.
        """
        reply = promise.get_reply()
        offer = reply.get_value('offer')
        self.webrtcbin.emit('set-local-description', offer, None)

        sdp_text = offer.sdp.as_text()
        log.info('[%s] sending offer (%d chars)', self.agent_id, len(sdp_text))
        asyncio.run_coroutine_threadsafe(
            self._ws_send({'type': 'offer', 'agent': self.agent_id, 'sdp': sdp_text}),
            self.loop,
        )

    def _on_ice_candidate(self, _webrtcbin, mline_index: int, candidate: str):
        """Forward each gathered ICE candidate to the browser via signaling."""
        asyncio.run_coroutine_threadsafe(
            self._ws_send({
                'type':  'ice',
                'agent': self.agent_id,
                'candidate': {
                    'candidate':     candidate,
                    'sdpMLineIndex': mline_index,
                },
            }),
            self.loop,
        )

    def add_ice_candidate(self, mline_index: int, candidate_str: str):
        """Apply an incoming browser ICE candidate to webrtcbin."""
        webrtcbin = self.webrtcbin  # snapshot for thread safety
        if webrtcbin and candidate_str:
            webrtcbin.emit('add-ice-candidate', mline_index, candidate_str)

    def _on_bus_error(self, _bus, msg):
        err, dbg = msg.parse_error()
        log.error('[%s] GStreamer error: %s (%s)', self.agent_id, err, dbg)

    def _on_bus_eos(self, _bus, _msg):
        log.warning('[%s] GStreamer EOS', self.agent_id)

    def apply_answer(self, sdp_str: str):
        """Set the browser's SDP answer as the remote description."""
        _, sdp = GstSdp.SDPMessage.new()
        GstSdp.sdp_message_parse_buffer(sdp_str.encode(), sdp)
        answer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.ANSWER, sdp
        )
        self.webrtcbin.emit('set-remote-description', answer, None)
        log.info('[%s] remote description (answer) applied', self.agent_id)

    def _reset_peer(self):
        """Tear down the pipeline so the next ROS2 frame rebuilds it for a new peer.

        Acquires _pipeline_lock to prevent push_frame from calling _build_pipeline
        concurrently while we are setting the old pipeline to NULL.
        """
        with self._pipeline_lock:
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            self.pipeline    = None
            self.appsrc      = None
            self.webrtcbin   = None
            self._pipe_ready = False
            self._pts        = 0
        log.info('[%s] pipeline reset — ready for next peer', self.agent_id)

    # ── WebSocket signaling ────────────────────────────────────────────────────

    async def _ws_send(self, obj: dict):
        if self.ws:
            try:
                await self.ws.send(json.dumps(obj))
            except Exception as e:
                log.warning('[%s] ws send failed: %s', self.agent_id, e)

    async def run_signaling(self):
        """Connect to the signaling server and handle messages forever."""
        while True:
            try:
                async with websockets.connect(self.signal_url) as ws:
                    self.ws = ws
                    log.info('[%s] connected to signaling server', self.agent_id)
                    await ws.send(json.dumps({'type': 'register', 'agent': self.agent_id}))

                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue

                        t = msg.get('type')

                        if t == 'consumer_joined':
                            log.info('[%s] consumer joined', self.agent_id)
                            self._reset_peer()
                            # Pipeline rebuilds on next ROS2 frame →
                            # on-negotiation-needed fires → offer + candidates sent

                        elif t == 'answer':
                            self.apply_answer(msg['sdp'])

                        elif t == 'ice':
                            cand = msg.get('candidate', {})
                            self.add_ice_candidate(
                                cand.get('sdpMLineIndex', 0),
                                cand.get('candidate', ''),
                            )

                        elif t == 'consumer_left':
                            log.info('[%s] consumer left', self.agent_id)
                            self._reset_peer()

            except Exception as e:
                log.warning('[%s] signaling error: %s — retrying in 3s', self.agent_id, e)
                self.ws = None
                await asyncio.sleep(3)

    async def run(self):
        self.loop = asyncio.get_event_loop()
        await self.run_signaling()


# ── ROS2 camera node ───────────────────────────────────────────────────────────

class CameraNode(Node):
    def __init__(self, agent_id: str, producer: GstWebRTCProducer):
        super().__init__(f'webrtc_{agent_id}')
        topic = f'/{agent_id}/camera/image_raw'
        self.sub = self.create_subscription(
            Image, topic, producer.push_frame, 10
        )
        self.get_logger().info(f'Subscribed to {topic}')


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    import argparse
    ap = argparse.ArgumentParser(description='GStreamer WebRTC camera producer')
    ap.add_argument('agent',        help='Agent ID (robot1, robot2, drone1, drone2)')
    ap.add_argument('signal_url',   nargs='?', default=DEFAULT_SIGNAL_URL,
                    help=f'Signaling server URL (default: {DEFAULT_SIGNAL_URL})')
    ap.add_argument('--sw', action='store_true',
                    help='Use software x264enc instead of nvh264enc')
    args = ap.parse_args()

    Gst.init(None)

    # GLib main loop in a daemon thread (needed for GStreamer bus signals)
    glib_loop   = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, daemon=True)
    glib_thread.start()

    # asyncio event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    producer = GstWebRTCProducer(
        agent_id   = args.agent,
        signal_url = args.signal_url,
        use_hw     = not args.sw,
    )

    # rclpy in a daemon thread (safe: only push_frame called from it)
    rclpy.init()
    ros_node   = CameraNode(args.agent, producer)
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(ros_node), daemon=True
    )
    ros_thread.start()

    try:
        loop.run_until_complete(producer.run())
    except KeyboardInterrupt:
        pass
    finally:
        if producer.pipeline:
            producer.pipeline.set_state(Gst.State.NULL)
        glib_loop.quit()
        rclpy.shutdown()
        ros_thread.join(timeout=2)
        log.info('[%s] shut down', args.agent)


if __name__ == '__main__':
    main()
