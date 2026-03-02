# ViDeG — New Session Handoff Prompt
_Copy everything below this line into a new Claude Code session_

---

You are continuing development of **ViDeG** — a latency engineering testbed for
cloud-connected robot teleoperation. Working directory: `/home/georg/projects/ViDeG`

Read `NEXT_STEPS.md` in full before doing anything — it is the authoritative reference
for project state, architecture, all bugs fixed, startup procedures, and next actions.

## Where we are

**Phase 1 (local pipeline) — COMPLETE.**
**Phase 2 (WAN deployment) — COMPLETE.**

Full pipeline proven end-to-end:
- Gazebo sim → camera → JPEG → WebSocket → EC2 relay → browser video ✅
- Browser WASD → EC2 relay → edge server → ROS2 cmd_vel → robot moves ✅
- Live demo conducted with remote viewers via shared `console.html` file ✅

**Measured baselines:**
- Localhost via relay: **82ms Cmd RTT, ~1ms video**
- EC2 us-west-2 WAN: **125–150ms Cmd RTT** ← proof of concept proven

## EC2 Info

- **Elastic IP:** 44.253.251.75
- **Region:** us-west-2 (Oregon)
- **Instance:** t3.micro, Ubuntu 24.04
- **PEM key:** `/home/georg/projects/ViDeG/ViDeG.pem`
- **Relay:** running in `screen -S relay` session, venv at `~/venv`
- **SSH:** `ssh -i ~/projects/ViDeG/ViDeG.pem ubuntu@44.253.251.75`

## Immediate next action — WSS upgrade

`scripts/setup_ec2_nginx.sh` is already written and ready to run on EC2.

**Step 1 — Get a domain:**
- Free option: `44-253-251-75.sslip.io` (no signup, works with Let's Encrypt)
- Paid option: Namecheap/Route 53 ~$12/yr, A record → 44.253.251.75

**Step 2 — EC2 security group:** open ports 80 and 443

**Step 3 — Copy and run setup script on EC2:**
```bash
scp -i ~/projects/ViDeG/ViDeG.pem \
    ~/projects/ViDeG/scripts/setup_ec2_nginx.sh \
    ubuntu@44.253.251.75:~/

ssh -i ~/projects/ViDeG/ViDeG.pem ubuntu@44.253.251.75
bash setup_ec2_nginx.sh YOUR_DOMAIN YOUR_EMAIL
```

**Step 4 — Upload console.html to EC2:**
```bash
scp -i ~/projects/ViDeG/ViDeG.pem \
    ~/projects/ViDeG/web/console.html \
    ubuntu@44.253.251.75:~/videg-web/
```

**Step 5 — Update local files** (once domain is known):
- `scripts/start_edge.sh`: change `--cloud ws://44.253.251.75:8766` → `--cloud wss://YOUR_DOMAIN/ws-edge`
- `web/console.html`: change default WS URL → `wss://YOUR_DOMAIN/ws-browser`

**Step 6 — Restart relay on localhost only:**
```bash
python3 ~/videg_relay.py --host 127.0.0.1
```

**Result:** Anyone opens `https://YOUR_DOMAIN/console.html` from anywhere — no file sharing needed.

## Startup sequence (always this order)

```bash
# Kill old stuff first
kill $(lsof -ti:8765) 2>/dev/null; kill $(lsof -ti:8766) 2>/dev/null; kill $(lsof -ti:8080) 2>/dev/null
pkill -f "gz " 2>/dev/null; pkill -f "ros2" 2>/dev/null; sleep 2

# T1
bash /home/georg/projects/ViDeG/scripts/start_relay.sh

# T2 — wait for "unpause_physics data: true"
bash /home/georg/projects/ViDeG/scripts/start_sim.sh

# T3 — AFTER unpause appears
bash /home/georg/projects/ViDeG/scripts/start_image_bridge.sh

# T4
bash /home/georg/projects/ViDeG/scripts/start_edge.sh
```

Browser: `http://localhost:8080/console.html` → WS: `ws://44.253.251.75:8765`

## Critical things to know

- `LIBGL_ALWAYS_SOFTWARE=1` — Gazebo camera won't render headless on WSL2 without this
- `ROS_DOMAIN_ID=42` — must be in ALL terminals or ROS2 topics are invisible
- `TURTLEBOT3_MODEL=waffle`
- Image bridge MUST start AFTER sim spawns robot or camera won't publish
- EC2 relay needs `source ~/venv/bin/activate` before `python3 ~/videg_relay.py`
- If port 8766 already in use: `kill $(lsof -ti:8766)`

## Architecture (current)

```
Browser → ws://44.253.251.75:8765 → [EC2 videg_relay.py] → ws://44.253.251.75:8766 ← edge server
                                                                        ↕ ROS2 domain 42
                                                                   Gazebo sim tb3_2
```

## Real hardware path (future)

Pi 5 or Jetson Orin Nano replaces WSL2 edge — **same code, no cloud changes:**
```bash
# On Pi 5 (ROS2 + websockets installed):
python3 videg_console_server.py --cloud wss://YOUR_DOMAIN/ws-edge
# Motor driver subscribes to /cmd_vel locally
```

## Roadmap after WSS

1. WSS — nginx + Let's Encrypt ← NEXT
2. Add robot 2 — uncomment `tb3_1` in `launch/fleet_sim.launch.py` FLEET list
3. WebRTC video — lower latency on congested WAN
4. Nav2 goal button — click waypoint, robot navigates autonomously
5. Real edge hardware — Pi 5 or Jetson Orin Nano 8GB (~$500)
6. Fleet relay redesign — N robots with per-robot ID routing

Full details in `NEXT_STEPS.md`.
