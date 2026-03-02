#!/usr/bin/env bash
# start_webrtc.sh — Launch WebRTC signaling server + 4 GStreamer agent producers
#
# Run AFTER start_sim.sh (Gazebo + rosbridge must be up and publishing image_raw).
#
# Usage:
#   chmod +x /opt/webrtc/start_webrtc.sh
#   /opt/webrtc/start_webrtc.sh            # NVENC (requires NVIDIA driver)
#   /opt/webrtc/start_webrtc.sh --sw       # software x264 fallback
#
# EC2 security group must allow:
#   TCP 8765 (inbound) — signaling WebSocket
#   UDP 49152-65535 (inbound) — WebRTC ICE media
#   UDP 19302 (outbound to 0.0.0.0/0) — STUN (usually open by default)
#
# One-time EC2 install (Ubuntu 24.04):
#   sudo apt-get install -y \
#       gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
#       gstreamer1.0-plugins-bad  gstreamer1.0-plugins-ugly \
#       gstreamer1.0-tools        python3-gi gir1.2-gstreamer-1.0 \
#       gir1.2-gst-plugins-bad-1.0
#   pip3 install websockets
#   # NVENC: requires nvidia-driver-535+ (see ec2_install_nvidia.sh)

set -euo pipefail

WEBRTC_DIR="/opt/webrtc"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SIGNAL_URL="ws://localhost:8765"
SW_FLAG=""

# Parse --sw flag
for arg in "$@"; do
    [[ "$arg" == "--sw" ]] && SW_FLAG="--sw" && echo "[start_webrtc] Using software encoder (x264)"
done

# ── 1. Stop old sessions ──────────────────────────────────────────────────────
echo "[start_webrtc] Stopping old sessions..."
for S in webrtc_signal webrtc_robot1 webrtc_robot2 webrtc_drone1 webrtc_drone2; do
    screen -S "$S" -X quit 2>/dev/null && echo "  killed: $S" || true
done
pkill -9 -f 'signaling.py'       2>/dev/null || true
pkill -9 -f 'gst_webrtc_agent.py' 2>/dev/null || true
sleep 2

# ── 2. Start signaling server ─────────────────────────────────────────────────
echo "[start_webrtc] Starting signaling server on port 8765..."
screen -dmS webrtc_signal bash -c "
    python3 ${WEBRTC_DIR}/signaling.py
    echo '[webrtc_signal] exited'
    exec bash
"
sleep 2  # let it bind

# ── 3. Start GStreamer agents ─────────────────────────────────────────────────
echo "[start_webrtc] Starting GStreamer WebRTC agents..."

start_agent() {
    local AGENT="$1"
    local SESSION="webrtc_${AGENT}"
    screen -dmS "$SESSION" bash -c "
        source ${ROS_SETUP}
        python3 ${WEBRTC_DIR}/gst_webrtc_agent.py ${AGENT} ${SIGNAL_URL} ${SW_FLAG}
        echo '[${SESSION}] exited'
        exec bash
    "
    echo "  started: $SESSION"
}

start_agent robot1
start_agent robot2
start_agent drone1
start_agent drone2

sleep 1

# ── 4. Done ───────────────────────────────────────────────────────────────────
EC2_IP=$(curl -sf http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null || echo "<EC2_IP>")

echo ""
echo "[start_webrtc] ✓ All WebRTC sessions launched:"
screen -ls | grep webrtc || true
echo ""
echo "[start_webrtc] Signaling server: ws://${EC2_IP}:8765"
echo "[start_webrtc] Dashboard:        http://${EC2_IP}"
echo ""
echo "[start_webrtc] Verify signaling (from EC2):"
echo "  screen -r webrtc_signal"
echo "  screen -r webrtc_robot1"
