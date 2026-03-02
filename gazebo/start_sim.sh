#!/usr/bin/env bash
# start_sim.sh — GeoCloud C2 Demo: launch full Gazebo stack on EC2
#
# Prerequisites (run once after instance upgrade to g4dn.xlarge):
#   See ec2_install.sh or the comments at the bottom of this file.
#
# Usage:
#   chmod +x /opt/gazebo/start_sim.sh
#   /opt/gazebo/start_sim.sh
#
# To check status after launch:
#   screen -ls
#   ros2 topic hz /robot1/odom
#   ros2 topic hz /robot1/camera/compressed

set -euo pipefail

ROS_SETUP="/opt/ros/jazzy/setup.bash"
GZ_DIR="/opt/gazebo"
SDF_FILE="${GZ_DIR}/c2_arena.sdf"
BRIDGE_CFG="${GZ_DIR}/bridge.yaml"

# ── 0. Sanity checks ──────────────────────────────────────────────────────────
echo "[start_sim] Checking prerequisites..."
[[ -f "$ROS_SETUP"  ]] || { echo "ERROR: ROS2 Jazzy not found at $ROS_SETUP"; exit 1; }
[[ -f "$SDF_FILE"   ]] || { echo "ERROR: SDF not found: $SDF_FILE"; exit 1; }
[[ -f "$BRIDGE_CFG" ]] || { echo "ERROR: bridge.yaml not found: $BRIDGE_CFG"; exit 1; }

command -v gz    >/dev/null 2>&1 || { echo "ERROR: gz not found — install gz-harmonic"; exit 1; }
command -v screen >/dev/null 2>&1 || { echo "ERROR: screen not installed"; exit 1; }

# ── 1. Kill existing screen sessions and orphaned processes ───────────────────
echo "[start_sim] Stopping old sessions..."
for SESSION in gazebo gz_bridge img_robot1 img_robot2 img_drone1 img_drone2 rosbridge \
               sim_robot1 sim_robot2 sim_drone1 sim_drone2 relay; do
    screen -S "$SESSION" -X quit 2>/dev/null && echo "  killed: $SESSION" || true
done
# Kill any lingering gz-sim processes that outlive their screen session
pkill -9 -f 'gz sim' 2>/dev/null && echo "  killed: orphaned gz sim processes" || true
pkill -9 -f 'parameter_bridge' 2>/dev/null || true
pkill -9 -f 'image_transport' 2>/dev/null || true
pkill -9 -f 'rosbridge' 2>/dev/null || true
sleep 3

# ── 2. Start gz-sim headless ─────────────────────────────────────────────────
echo "[start_sim] Starting gz-sim (headless)..."
screen -dmS gazebo bash -c "
    # Do NOT source ROS here — ROS sets GZ_CONFIG_PATH to vendor dirs
    # which hides the system gz-sim subcommand.
    # gz-sim does not need ROS to run.
    export GZ_SIM_RESOURCE_PATH=${GZ_DIR}
    # NVIDIA GPU EGL rendering — glvnd auto-selects NVIDIA over Mesa (10_nvidia.json)
    # No MESA_GL_VERSION_OVERRIDE needed; NVIDIA driver handles EGL headlessly.
    # -s = server only (no Qt GUI), --headless-rendering = EGL camera sensors
    /usr/bin/gz sim -s --headless-rendering ${SDF_FILE} -r
    echo '[gazebo] gz-sim exited'
    exec bash
"

echo "[start_sim] Waiting 10s for gz-sim to initialise world..."
sleep 10

# ── 3. Start ros_gz_bridge ───────────────────────────────────────────────────
echo "[start_sim] Starting ros_gz_bridge..."
screen -dmS gz_bridge bash -c "
    source ${ROS_SETUP}
    ros2 run ros_gz_bridge parameter_bridge \
        --ros-args -p config_file:=${BRIDGE_CFG}
    echo '[gz_bridge] exited'
    exec bash
"

echo "[start_sim] Waiting 4s for bridge to connect..."
sleep 4

# ── 4. image_transport republish: raw Image → CompressedImage (JPEG) ─────────
# One process per camera topic (320×240 @ 10fps is very light)
echo "[start_sim] Starting image_transport republish × 4..."

republish_camera() {
    local AGENT="$1"
    local SESSION="img_${AGENT}"
    screen -dmS "$SESSION" bash -c "
        source ${ROS_SETUP}
        ros2 run image_transport republish raw compressed \
            --ros-args \
            -r in:=/${AGENT}/camera/image_raw \
            -r out/compressed:=/${AGENT}/camera/compressed \
            -p jpeg_quality:=95
        echo '[${SESSION}] exited'
        exec bash
    "
    echo "  started: $SESSION (/${AGENT}/camera/compressed)"
}

republish_camera robot1
republish_camera robot2
republish_camera drone1
republish_camera drone2

sleep 2

# ── 5. Start rosbridge WebSocket server ──────────────────────────────────────
echo "[start_sim] Starting rosbridge on port 9090..."
screen -dmS rosbridge bash -c "
    source ${ROS_SETUP}
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
    echo '[rosbridge] exited'
    exec bash
"

# ── 6. Done ───────────────────────────────────────────────────────────────────
sleep 3
echo ""
echo "[start_sim] ✓ All sessions launched. Summary:"
screen -ls
echo ""
echo "[start_sim] Verify topics are live (after ~15s total):"
echo "  source ${ROS_SETUP}"
echo "  ros2 topic hz /robot1/odom              # expect ~10 Hz"
echo "  ros2 topic hz /robot1/camera/compressed # expect ~10 Hz"
echo "  ros2 topic echo /robot1/odom --once"
echo ""
echo "[start_sim] Browser: http://44.253.251.75/robot1"

# ══════════════════════════════════════════════════════════════════════════════
# ONE-TIME EC2 INSTALL (t3.large, Ubuntu 24.04 / ROS2 Jazzy)
# Run these manually after upgrading the instance — NOT part of start_sim.sh
# ══════════════════════════════════════════════════════════════════════════════
#
# # Gazebo Harmonic apt repo
# sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
#     -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
#     http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
#     | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
# sudo apt-get update
# sudo apt-get install -y gz-harmonic
#
# # Mesa software rendering (headless EGL, no GPU required)
# sudo apt-get install -y \
#     libgl1-mesa-dri libgles2-mesa-dev libegl1-mesa-dev \
#     mesa-utils libglx-mesa0
#
# # ROS-gz bridge + image transport
# sudo apt-get install -y \
#     ros-jazzy-ros-gz \
#     ros-jazzy-ros-gz-sim \
#     ros-jazzy-ros-gz-bridge \
#     ros-jazzy-image-transport \
#     ros-jazzy-image-transport-plugins \
#     ros-jazzy-compressed-image-transport
#
# # Deploy gazebo files
# sudo mkdir -p /opt/gazebo
# sudo cp c2_arena.sdf bridge.yaml start_sim.sh /opt/gazebo/
# sudo chmod +x /opt/gazebo/start_sim.sh
#
# # EC2 Security Group — ensure port 9090 (TCP) is open from 0.0.0.0/0
