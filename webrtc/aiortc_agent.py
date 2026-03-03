#!/usr/bin/env python3
"""
aiortc_agent.py — aiortc WebRTC producer for one C2 camera agent.

Replaces gst_webrtc_agent.py. Uses aiortc (pure-Python WebRTC) instead of
GStreamer webrtcbin to avoid libnice/ICE quirks.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 aiortc_agent.py robot1
  python3 aiortc_agent.py robot1 ws://localhost:8765
"""

import sys, asyncio, json, logging, threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from aiortc.sdp import candidate_from_sdp
import av
import websockets

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s [%(name)s] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('aiortc')

DEFAULT_SIGNAL_URL = 'ws://localhost:8765'


# ── Video track ────────────────────────────────────────────────────────────────

class RosVideoTrack(VideoStreamTrack):
    """Feeds ROS2 Image messages into an aiortc VideoStreamTrack."""
    kind = 'video'

    def __init__(self):
        super().__init__()
        self._queue = None   # asyncio.Queue — created once loop is running
        self._loop  = None

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop  = loop
        self._queue = asyncio.Queue(maxsize=2)

    def push_frame(self, msg: Image):
        if self._loop is None or self._queue is None:
            return
        if msg.encoding in ('bgr8', 'BGR8'):
            arr = np.frombuffer(msg.data, dtype='uint8').reshape(msg.height, msg.width, 3)
            rgb = arr[:, :, ::-1].copy()
        else:
            rgb = np.frombuffer(msg.data, dtype='uint8').reshape(msg.height, msg.width, 3).copy()
        asyncio.run_coroutine_threadsafe(self._enqueue(rgb), self._loop)

    async def _enqueue(self, rgb):
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        await self._queue.put(rgb)

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        rgb = await self._queue.get()
        frame = av.VideoFrame.from_ndarray(rgb, format='rgb24')
        frame.pts       = pts
        frame.time_base = time_base
        return frame


# ── Agent ──────────────────────────────────────────────────────────────────────

class AiortcAgent:
    def __init__(self, agent_id: str, signal_url: str):
        self.agent_id   = agent_id
        self.signal_url = signal_url
        self.track: RosVideoTrack | None = None  # always fresh per PC
        self.pc:  RTCPeerConnection | None = None
        self.ws         = None

    def push_frame(self, msg: Image):
        if self.track is not None:
            self.track.push_frame(msg)

    async def _reset_peer(self):
        if self.pc:
            await self.pc.close()
            self.pc = None

    async def _new_offer(self):
        await self._reset_peer()

        # Create a fresh track for every new PC.
        # pc.close() calls track.stop() internally, setting readyState="ended".
        # Reusing a stopped track causes MediaStreamError on the next recv() call.
        track = RosVideoTrack()
        track.set_loop(asyncio.get_event_loop())
        self.track = track   # push_frame() now routes to the new track

        config = RTCConfiguration(iceServers=[
            RTCIceServer(urls=['stun:stun.l.google.com:19302']),
        ])
        pc = RTCPeerConnection(configuration=config)
        self.pc = pc
        pc.addTrack(track)

        # Forward gathered ICE candidates to browser
        @pc.on('icecandidate')
        async def on_ice(candidate):
            if candidate is None or self.ws is None:
                return
            try:
                await self.ws.send(json.dumps({
                    'type':  'ice',
                    'agent': self.agent_id,
                    'candidate': {
                        'candidate':     f'candidate:{candidate.to_sdp()}',
                        'sdpMLineIndex': 0,
                    },
                }))
            except Exception:
                pass

        @pc.on('connectionstatechange')
        async def on_state():
            log.info('[%s] connection state → %s', self.agent_id, pc.connectionState)

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        if self.ws:
            sdp = pc.localDescription.sdp
            log.info('[%s] sending offer (%d chars)', self.agent_id, len(sdp))
            await self.ws.send(json.dumps({
                'type':  'offer',
                'agent': self.agent_id,
                'sdp':   sdp,
            }))

    async def run_signaling(self):
        while True:
            try:
                async with websockets.connect(self.signal_url) as ws:
                    self.ws = ws
                    log.info('[%s] connected to signaling', self.agent_id)
                    await ws.send(json.dumps({'type': 'register', 'agent': self.agent_id}))

                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue

                        t = msg.get('type')

                        if t == 'consumer_joined':
                            log.info('[%s] consumer joined — sending offer', self.agent_id)
                            await self._new_offer()

                        elif t == 'answer' and self.pc:
                            desc = RTCSessionDescription(sdp=msg['sdp'], type='answer')
                            await self.pc.setRemoteDescription(desc)
                            log.info('[%s] answer applied', self.agent_id)

                        elif t == 'ice' and self.pc:
                            cand_dict = msg.get('candidate', {})
                            sdp_str   = cand_dict.get('candidate', '')
                            if sdp_str:
                                # Strip leading 'candidate:' prefix if present
                                raw_sdp = sdp_str[10:] if sdp_str.startswith('candidate:') else sdp_str
                                try:
                                    c = candidate_from_sdp(raw_sdp)
                                    await self.pc.addIceCandidate(c)
                                except Exception as e:
                                    log.debug('[%s] addIceCandidate: %s', self.agent_id, e)

                        elif t == 'consumer_left':
                            log.info('[%s] consumer left', self.agent_id)
                            await self._reset_peer()

            except Exception as e:
                log.warning('[%s] signaling error: %s — retry in 3s', self.agent_id, e)
                self.ws = None
                await asyncio.sleep(3)

    async def run(self):
        await self.run_signaling()


# ── ROS2 node ──────────────────────────────────────────────────────────────────

class CameraNode(Node):
    def __init__(self, agent_id: str, agent: AiortcAgent):
        super().__init__(f'aiortc_{agent_id}')
        topic = f'/{agent_id}/camera/image_raw'
        self.sub = self.create_subscription(Image, topic, agent.push_frame, 10)
        self.get_logger().info(f'Subscribed to {topic}')


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('agent')
    ap.add_argument('signal_url', nargs='?', default=DEFAULT_SIGNAL_URL)
    args = ap.parse_args()

    rclpy.init()
    agent    = AiortcAgent(args.agent, args.signal_url)
    ros_node = CameraNode(args.agent, agent)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    ros_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(agent.run())
    except KeyboardInterrupt:
        pass
    finally:
        if agent.pc:
            loop.run_until_complete(agent.pc.close())
        rclpy.shutdown()
        ros_thread.join(timeout=2)
        log.info('[%s] shut down', args.agent)


if __name__ == '__main__':
    main()
