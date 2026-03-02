#!/usr/bin/env python3
"""
videg_relay.py  v1.0  (deploy on EC2 or run locally for pre-flight test)
════════════════════════════════════════════════════════════════════════════
ViDeG Cloud Relay — WebSocket broker

Routes messages between the edge server and operator browsers.
Both sides connect TO this relay, so neither needs open inbound ports.

  Edge server  →  ws://RELAY_HOST:8766  (one persistent connection)
  Browser(s)   →  ws://RELAY_HOST:8765  (N operator connections)

Message routing:
  Edge  → all browsers : binary JPEG frames + JSON telemetry (broadcast)
  Browser → edge       : JSON cmd_vel commands (forwarded)

Usage:
  python3 scripts/videg_relay.py
  python3 scripts/videg_relay.py --edge-port 8766 --browser-port 8765

Local pre-flight test (3 terminals):
  # T1 — relay
  python3 scripts/videg_relay.py

  # T2 — edge (note --cloud flag)
  source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
  python3 scripts/videg_console_server.py --cloud ws://localhost:8766

  # T3 — browser (console.html is served by edge HTTP server on :8080)
  open http://localhost:8080/console.html
  # Connect to ws://localhost:8765  (not 8765 on localhost — change the WS URL field)
"""

import argparse
import asyncio
import logging
import sys
import time

try:
    import websockets
    import websockets.exceptions
except ImportError:
    print('[relay] ERROR: websockets not installed.')
    print('  Run:  pip install "websockets>=12.0"')
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [relay] %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('relay')


# ── Shared state (asyncio single-threaded — no locks needed) ─────────────────
_edge_ws      = None   # the one registered edge WebSocket connection
_browsers: set = set() # all connected browser WebSocket objects
_msg_count    = 0      # total messages forwarded (diagnostics)
_edge_conn_t  = 0.0    # monotonic time edge last connected


# ── Edge handler ─────────────────────────────────────────────────────────────
async def edge_handler(websocket) -> None:
    """Accept the edge server, fan its frames+telemetry out to all browsers."""
    global _edge_ws, _msg_count, _edge_conn_t

    if _edge_ws is not None:
        log.warning('Second edge connection rejected — only one edge allowed at a time.')
        await websocket.close(1008, 'Edge slot occupied')
        return

    _edge_ws     = websocket
    _edge_conn_t = time.monotonic()
    remote       = websocket.remote_address
    log.info(f'Edge connected from {remote}  (browsers online: {len(_browsers)})')

    try:
        async for msg in websocket:
            if not _browsers:
                continue
            # Broadcast to all browsers; ignore individual send errors —
            # each browser's own handler removes it from _browsers on close.
            await asyncio.gather(
                *[b.send(msg) for b in list(_browsers)],
                return_exceptions=True,
            )
            _msg_count += 1
    except websockets.exceptions.ConnectionClosed as exc:
        log.info(f'Edge disconnected: {exc}')
    finally:
        _edge_ws = None
        log.info('Edge slot freed — waiting for edge to reconnect.')


# ── Browser handler ───────────────────────────────────────────────────────────
async def browser_handler(websocket) -> None:
    """Accept an operator browser; forward its JSON commands to the edge."""
    _browsers.add(websocket)
    remote = websocket.remote_address
    log.info(f'Browser connected: {remote}  (total: {len(_browsers)})')

    if _edge_ws is None:
        log.warning('  ↳ No edge connected yet — commands will be dropped until edge arrives.')

    try:
        async for msg in websocket:
            if not isinstance(msg, str):
                continue   # browsers only send text JSON commands
            if _edge_ws is not None:
                try:
                    await _edge_ws.send(msg)
                except websockets.exceptions.ConnectionClosed:
                    pass   # edge gone; edge_handler will clear _edge_ws
    except websockets.exceptions.ConnectionClosed as exc:
        log.info(f'Browser disconnected: {remote} — {exc}')
    finally:
        _browsers.discard(websocket)
        log.info(f'Browser removed: {remote}  (remaining: {len(_browsers)})')


# ── Diagnostics loop ──────────────────────────────────────────────────────────
async def diagnostics_loop() -> None:
    global _msg_count
    while True:
        await asyncio.sleep(30)
        uptime = (
            f'connected ({time.monotonic() - _edge_conn_t:.0f}s)'
            if _edge_ws else 'DISCONNECTED'
        )
        log.info(
            f'Status — edge: {uptime}'
            f' | browsers: {len(_browsers)}'
            f' | msgs forwarded: {_msg_count}'
        )


# ── Main ──────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description='ViDeG WebSocket relay — deploy on EC2 or run locally for testing',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--host',         default='0.0.0.0',
                        help='Bind host')
    parser.add_argument('--edge-port',    type=int, default=8766,
                        help='Port for edge server registration')
    parser.add_argument('--browser-port', type=int, default=8765,
                        help='Port for operator browser connections')
    args = parser.parse_args()

    print(f'\n{"═"*60}')
    print(f'  ViDeG Cloud Relay  v1.0')
    print(f'{"═"*60}')
    print(f'  Edge port:    {args.host}:{args.edge_port}')
    print(f'    ← edge server: python3 scripts/videg_console_server.py \\')
    print(f'                       --cloud ws://RELAY_HOST:{args.edge_port}')
    print(f'  Browser port: {args.host}:{args.browser_port}')
    print(f'    ← browser WS URL: ws://RELAY_HOST:{args.browser_port}')
    print(f'{"═"*60}\n', flush=True)

    async def _serve() -> None:
        async with (
            websockets.serve(edge_handler,   args.host, args.edge_port),
            websockets.serve(browser_handler, args.host, args.browser_port),
        ):
            log.info(
                f'Relay listening — '
                f'edge on :{args.edge_port}, browsers on :{args.browser_port}'
            )
            await asyncio.gather(
                asyncio.Future(),   # run forever
                diagnostics_loop(),
            )

    try:
        asyncio.run(_serve())
    except KeyboardInterrupt:
        print('\n[relay] Shutting down.')


if __name__ == '__main__':
    main()
