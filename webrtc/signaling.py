#!/usr/bin/env python3
"""
signaling.py — WebRTC Signaling Server for GeoCloud C2 Dashboard

Bridges GStreamer/aiortc producers (one per agent) with browser consumers.
Runs on port 8765 as a plain WebSocket server.

Protocol (all messages are JSON):
  producer → server:  { type: "register", agent: "robot1" }
  consumer → server:  { type: "join",     agent: "robot1" }
  server   → producer:{ type: "consumer_joined" }
  producer → server:  { type: "offer",    agent: "robot1", sdp: "..." }
  server   → consumer:{ type: "offer",    agent: "robot1", sdp: "..." }
  consumer → server:  { type: "answer",   agent: "robot1", sdp: "..." }
  server   → producer:{ type: "answer",   sdp: "..." }
  server   → producer:{ type: "consumer_left" }  (on consumer disconnect)

Trickle ICE: offer and answer SDP are sent without candidates; each side
forwards ICE candidates individually as they are gathered, and the remote
peer adds them via add-ice-candidate as they arrive.

One active producer and one active consumer per agent.
A new connection silently replaces the previous one.
"""

import asyncio
import json
import logging

import websockets
from websockets.server import WebSocketServerProtocol

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s [signaling] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('signaling')

# agent_id → { "producer": ws | None, "consumer": ws | None }
rooms: dict[str, dict] = {}


def _room(agent: str) -> dict:
    if agent not in rooms:
        rooms[agent] = {'producer': None, 'consumer': None}
    return rooms[agent]


async def _safe_send(ws: WebSocketServerProtocol, payload: dict) -> bool:
    try:
        await ws.send(json.dumps(payload))
        return True
    except Exception:
        return False


async def handler(ws: WebSocketServerProtocol):
    agent: str | None = None
    role:  str | None = None  # "producer" | "consumer"

    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            t = msg.get('type')
            a = msg.get('agent')

            # ── Registration ───────────────────────────────────────────────────
            if t == 'register' and a:
                agent, role = a, 'producer'
                room = _room(agent)
                if room['producer']:
                    try:
                        await room['producer'].close()
                    except Exception:
                        pass
                room['producer'] = ws
                log.info('[%s] producer registered', agent)

            elif t == 'join' and a:
                agent, role = a, 'consumer'
                room = _room(agent)
                if room['consumer']:
                    try:
                        await room['consumer'].close()
                    except Exception:
                        pass
                room['consumer'] = ws
                log.info('[%s] consumer joined', agent)

                prod = room['producer']
                if prod:
                    ok = await _safe_send(prod, {'type': 'consumer_joined'})
                    if not ok:
                        room['producer'] = None

            # ── Forwarded signaling (offer / answer / ice) ─────────────────────
            elif t in ('offer', 'answer', 'ice') and agent:
                room = rooms.get(agent)  # pure lookup — don't create a room for unknown agents
                if not room:
                    continue
                if role == 'producer':
                    target = room['consumer']
                    if target:
                        ok = await _safe_send(target, msg)
                        if not ok:
                            room['consumer'] = None
                elif role == 'consumer':
                    target = room['producer']
                    if target:
                        ok = await _safe_send(target, msg)
                        if not ok:
                            room['producer'] = None

    except websockets.exceptions.ConnectionClosed:
        pass

    finally:
        if agent and agent in rooms:
            room = rooms[agent]
            if role == 'producer' and room['producer'] is ws:
                room['producer'] = None
                log.info('[%s] producer disconnected', agent)

            elif role == 'consumer' and room['consumer'] is ws:
                room['consumer'] = None
                log.info('[%s] consumer disconnected', agent)
                prod = room['producer']
                if prod:
                    ok = await _safe_send(prod, {'type': 'consumer_left'})
                    if not ok:
                        room['producer'] = None


async def main():
    log.info('Signaling server listening on ws://0.0.0.0:8765')
    async with websockets.serve(handler, '0.0.0.0', 8765):
        await asyncio.Future()  # run forever


if __name__ == '__main__':
    asyncio.run(main())
