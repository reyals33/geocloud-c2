#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════
#  GeoCloud Fleet Simulation — Demo Start Script  v2.1.0
#  Runs a pre-flight check, then launches the full stack in the right order.
#
#  Usage:
#    chmod +x demo_start.sh
#    ./demo_start.sh              # normal (Gazebo with GUI)
#    ./demo_start.sh --headless   # no Gazebo window (server/CI)
#    ./demo_start.sh --check-only # run pre-flight only, do not launch
#
#  What this script does:
#    1. Pre-flight: checks all required tools and packages are installed
#    2. Sets environment variables
#    3. Starts edge broker (Docker Compose) in background
#    4. Waits for MediaMTX and Prometheus to be ready
#    5. Launches the ROS2 fleet simulation in a new terminal
#    6. Prints quick-access URLs for browser and monitoring
#
#  Press Ctrl+C in this terminal to stop the edge broker.
#  The ROS2 simulation runs in its own terminal — close that window to stop it.
# ═══════════════════════════════════════════════════════════════════════
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="${REPO_DIR}/.venv"
VENV_PYTHON="${VENV}/bin/python3"
HEADLESS="false"
CHECK_ONLY="false"
GROUND_ONLY="false"

for arg in "$@"; do
  case "$arg" in
    --headless)     HEADLESS="true" ;;
    --check-only)   CHECK_ONLY="true" ;;
    --ground-only)  GROUND_ONLY="true" ;;
  esac
done

# ── Colours ────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  [OK]${NC}  $*"; }
warn() { echo -e "${YELLOW}  [WARN]${NC} $*"; }
fail() { echo -e "${RED}  [FAIL]${NC} $*"; PREFLIGHT_FAIL=1; }

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  GeoCloud Fleet Simulation v2.1.0 — Pre-Flight Check"
echo "═══════════════════════════════════════════════════════════"

PREFLIGHT_FAIL=0

# ── 0. Venv ──────────────────────────────────────────────────────────
echo ""
echo "[ Python Venv ]"
if [ -f "${VENV_PYTHON}" ]; then
  ok "Project venv: ${VENV}"
  # Activate for this shell session
  # shellcheck disable=SC1091
  source "${VENV}/bin/activate"
else
  fail "Project venv not found. Run once to create it:"
  fail "  python3 -m venv --system-site-packages ${VENV} && ${VENV}/bin/pip install -r ${REPO_DIR}/requirements.txt"
  exit 1
fi

# ── 1. System tools ─────────────────────────────────────────────────
echo ""
echo "[ System Tools ]"
command -v docker   &>/dev/null && ok "docker found"       || fail "docker not installed → sudo apt install docker.io"
command -v python3  &>/dev/null && ok "python3 found"      || fail "python3 not installed"
command -v ffmpeg   &>/dev/null && ok "ffmpeg found (host)" || warn "ffmpeg not on host (OK if only used in Docker)"

# Docker daemon running?
docker info &>/dev/null && ok "Docker daemon running" || fail "Docker daemon not running → sudo systemctl start docker"

# ── 2. ROS2 environment ─────────────────────────────────────────────
echo ""
echo "[ ROS2 Environment ]"
if [ -z "${ROS_DISTRO:-}" ]; then
  # Auto-detect installed distro (jazzy preferred, humble fallback)
  if   [ -f /opt/ros/jazzy/setup.bash ];  then ROS_SETUP=/opt/ros/jazzy/setup.bash
  elif [ -f /opt/ros/humble/setup.bash ]; then ROS_SETUP=/opt/ros/humble/setup.bash
  else
    fail "No ROS2 install found under /opt/ros/. See SETUP_MANUAL.md §2.1"
    exit 1
  fi
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
  warn "ROS2 was not sourced — sourced ${ROS_SETUP} for this session."
  warn "Add 'source ${ROS_SETUP}' to ~/.bashrc for persistence."
else
  ok "ROS2 ${ROS_DISTRO} sourced"
fi

[ "${ROS_DOMAIN_ID:-}" = "42" ] && ok "ROS_DOMAIN_ID=42" || {
  warn "ROS_DOMAIN_ID is '${ROS_DOMAIN_ID:-unset}', expected 42 — setting now"
  export ROS_DOMAIN_ID=42
}

# ── 3. Required ROS2 packages ────────────────────────────────────────
echo ""
echo "[ ROS2 Packages ]"
DISTRO="${ROS_DISTRO:-jazzy}"
# Check via ament_index filesystem — reliable even when ros2 CLI has env issues
check_pkg() {
  local pkg="$1"
  local apt_suffix="${2:-$(echo "$pkg" | tr '_' '-')}"
  # Look in all known ROS2 prefix share dirs
  for prefix in /opt/ros/"${DISTRO}" ${AMENT_PREFIX_PATH//:/ }; do
    [ -d "${prefix}/share/${pkg}" ] && { ok "${pkg}"; return; }
  done
  fail "${pkg} not installed → sudo apt install ros-${DISTRO}-${apt_suffix}"
}
# Gazebo: Humble uses gazebo_ros (Classic); Jazzy uses ros_gz (Harmonic)
if [ "${DISTRO}" = "jazzy" ] || [ "${DISTRO}" = "iron" ]; then
  ros2 pkg list 2>/dev/null | grep -q "ros_gz_sim\|gazebo_ros" \
    && ok "gazebo bridge (ros_gz_sim)" \
    || fail "ros_gz_sim not installed → sudo apt install ros-${DISTRO}-ros-gz"
else
  check_pkg gazebo_ros gazebo-ros-pkgs
fi
check_pkg turtlebot3_gazebo       turtlebot3-gazebo
check_pkg turtlebot3_navigation2  turtlebot3-navigation2
check_pkg slam_toolbox
check_pkg nav2_bringup
check_pkg mavros
check_pkg image_transport
check_pkg tf2_ros
{ ls /opt/ros/"${DISTRO}"/share/ 2>/dev/null | grep -q "explore"; } \
  && ok "explore_lite" \
  || warn "explore_lite not found → sudo apt install ros-${DISTRO}-explore-lite  (optional; exploration disabled without it)"

# ── 4. Python dependencies (checked inside venv) ─────────────────────
echo ""
echo "[ Python Dependencies (venv) ]"
"${VENV_PYTHON}" -c "from scipy.spatial import cKDTree" 2>/dev/null \
  && ok "scipy (venv)" \
  || fail "scipy missing in venv → ${VENV}/bin/pip install scipy"
"${VENV_PYTHON}" -c "import prometheus_client" 2>/dev/null \
  && ok "prometheus_client (venv)" \
  || fail "prometheus_client missing in venv → ${VENV}/bin/pip install prometheus-client"

# ── 5. PX4 + micro-XRCE-DDS ─────────────────────────────────────────
echo ""
echo "[ PX4 / MAVROS ]"
PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
if [ "${GROUND_ONLY}" = "true" ]; then
  warn "PX4 check skipped (--ground-only mode — drones disabled)"
else
  if [ -d "${PX4_DIR}" ]; then
    IRIS_SDF="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf"
    [ -f "${IRIS_SDF}" ] && ok "PX4 Iris SDF found" || warn "Iris SDF not found — drone spawn may fail"
    ok "PX4 directory: ${PX4_DIR}"
  else
    fail "PX4-Autopilot not found at ${PX4_DIR}. See SETUP_MANUAL.md §2.3"
  fi
  command -v MicroXRCEAgent &>/dev/null && ok "MicroXRCEAgent found" \
    || fail "MicroXRCEAgent not installed → see SETUP_MANUAL.md §2.3"
fi

# ── 6. VINS-Fusion workspace ─────────────────────────────────────────
echo ""
echo "[ VINS-Fusion Workspace ]"
if [ -d "${HOME}/vins_ws/install/vins_fusion" ] || \
   [ -d "/opt/ros/${DISTRO}/share/vins_fusion" ]; then
  ok "vins_fusion package found"
else
  warn "vins_fusion not found — build from ~/vins_ws (see SETUP_MANUAL.md §2.6)"
  warn "Drone VIO will not function without it — ground robot demo still works."
fi

# ── 7. FastDDS profile file ──────────────────────────────────────────
echo ""
echo "[ Configuration ]"
[ -f "${REPO_DIR}/config/fastdds_profile.xml" ] \
  && ok "fastdds_profile.xml" \
  || fail "config/fastdds_profile.xml missing"
[ -f "${REPO_DIR}/config/mediamtx.yml" ] \
  && ok "mediamtx.yml" \
  || fail "config/mediamtx.yml missing"
[ -f "${REPO_DIR}/config/prometheus.yml" ] \
  && ok "prometheus.yml" \
  || fail "config/prometheus.yml missing"

# ── Pre-flight summary ───────────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════════"
if [ "${PREFLIGHT_FAIL}" -ne 0 ]; then
  echo -e "${RED}  PRE-FLIGHT FAILED — fix the [FAIL] items above before launching.${NC}"
  echo -e "${RED}  See SETUP_MANUAL.md for step-by-step install instructions.${NC}"
  echo "═══════════════════════════════════════════════════════════"
  exit 1
else
  echo -e "${GREEN}  PRE-FLIGHT PASSED — system ready to launch.${NC}"
  echo "═══════════════════════════════════════════════════════════"
fi

[ "${CHECK_ONLY}" = "true" ] && { echo "  --check-only flag set, exiting without launch."; exit 0; }

# ── Set environment variables ─────────────────────────────────────────
export ROS_DOMAIN_ID=42
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_DIR}/config/fastdds_profile.xml"
export RMW_FASTRTPS_USE_QOS_FROM_ENV=0
export PX4_DIR="${PX4_DIR:-${HOME}/PX4-Autopilot}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}:${PX4_DIR}/build/px4_sitl_default/build_gazebo-classic"
[ -f "${HOME}/vins_ws/install/setup.bash" ] && source "${HOME}/vins_ws/install/setup.bash"

echo ""
echo "[ Step 1/3 ] Building Docker images (first run only — cached after that)..."
docker compose -f "${REPO_DIR}/docker/docker-compose.edge.yml" build --quiet
echo -e "${GREEN}  Docker images ready.${NC}"

# ── Start edge broker ────────────────────────────────────────────────
echo ""
echo "[ Step 2/3 ] Starting edge broker stack (Docker Compose)..."
docker compose -f "${REPO_DIR}/docker/docker-compose.edge.yml" up -d
echo ""
echo "  Waiting for MediaMTX (RTSP server)..."
for i in $(seq 1 20); do
  if curl -s http://localhost:8889 &>/dev/null || \
     docker compose -f "${REPO_DIR}/docker/docker-compose.edge.yml" \
       logs rtsp_server 2>/dev/null | grep -q "listener opened"; then
    ok "MediaMTX ready"
    break
  fi
  sleep 2
  [ "$i" -eq 20 ] && warn "MediaMTX may not be ready yet — continuing anyway"
done

echo "  Waiting for Prometheus..."
for i in $(seq 1 15); do
  if curl -s http://localhost:9090/-/ready &>/dev/null; then
    ok "Prometheus ready"
    break
  fi
  sleep 2
  [ "$i" -eq 15 ] && warn "Prometheus may not be ready yet"
done

# ── Launch ROS2 simulation in new terminal ─────────────────────────
echo ""
echo "[ Step 3/3 ] Launching ROS2 fleet simulation..."

ROS_SETUP_FILE="/opt/ros/${ROS_DISTRO}/setup.bash"
ROS_LAUNCH_CMD="source ${ROS_SETUP_FILE}"
[ -f "${HOME}/vins_ws/install/setup.bash" ] && ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && source ${HOME}/vins_ws/install/setup.bash"
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && source ${VENV}/bin/activate"
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && export ROS_DOMAIN_ID=42"
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && export GAZEBO_MODEL_PATH='${GAZEBO_MODEL_PATH}'"
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && export GAZEBO_PLUGIN_PATH='${GAZEBO_PLUGIN_PATH}'"
LAUNCH_ARGS="headless:=${HEADLESS}"
if [ "${GROUND_ONLY}" = "true" ]; then
  LAUNCH_ARGS="${LAUNCH_ARGS} enable_drones:=false"
  export GEOCLOUDS_ENABLE_DRONES=false
  ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && export GEOCLOUDS_ENABLE_DRONES=false"
fi
ROS_LAUNCH_CMD="${ROS_LAUNCH_CMD} && ros2 launch ${REPO_DIR}/launch/fleet_sim.launch.py ${LAUNCH_ARGS}"

# Try to detect the terminal emulator available and open a new window
if command -v gnome-terminal &>/dev/null; then
  gnome-terminal --title="GeoCloud Fleet Sim" -- bash -c "${ROS_LAUNCH_CMD}; exec bash"
elif command -v xterm &>/dev/null; then
  xterm -T "GeoCloud Fleet Sim" -e "bash -c '${ROS_LAUNCH_CMD}; exec bash'" &
elif command -v konsole &>/dev/null; then
  konsole --new-tab -e "bash -c '${ROS_LAUNCH_CMD}; exec bash'" &
elif command -v tmux &>/dev/null; then
  tmux new-window -n "fleet-sim" "bash -c '${ROS_LAUNCH_CMD}; exec bash'"
  ok "ROS2 fleet launched in tmux window 'fleet-sim' (switch with Ctrl+B n)"
else
  echo ""
  warn "No supported terminal emulator found (gnome-terminal/xterm/konsole/tmux)."
  warn "Run this manually in a separate terminal:"
  echo ""
  echo "  ${ROS_LAUNCH_CMD}"
  echo ""
fi

# ── Print quick-access URLs ──────────────────────────────────────────
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  GeoCloud Fleet — LIVE URLS"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "  VIDEO STREAMS (WebRTC, <100ms latency):"
echo "    http://localhost:8889/tb3_1    ← TurtleBot3 #1"
echo "    http://localhost:8889/tb3_2    ← TurtleBot3 #2"
echo "    http://localhost:8889/drone_1  ← Drone #1 (Iris)"
echo "    http://localhost:8889/drone_2  ← Drone #2 (Iris)"
echo ""
echo "  RTSP streams (VLC/ffplay):"
echo "    rtsp://localhost:8554/tb3_1"
echo "    rtsp://localhost:8554/drone_1"
echo ""
echo "  MONITORING:"
echo "    http://localhost:3000          ← Grafana  (admin / fleet_admin)"
echo "    http://localhost:9090          ← Prometheus"
echo "    http://localhost:9091/metrics  ← Raw latency metrics"
echo ""
echo "  STARTUP TIMELINE (automatic, ~35 seconds):"
echo "    0s  — Gazebo loads turtlebot3_world.world"
echo "    +5s — Robots spawn"
echo "    +15s— Nav2 + SLAM start"
echo "    +25s— Fusion + drone offboard"
echo "    +35s— Cameras + exploration + monitoring"
echo ""
echo "  SHUTDOWN:  Ctrl+C here (stops Docker), close sim terminal (stops ROS2)"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Keep edge broker in foreground ──────────────────────────────────
echo "Edge broker logs (Ctrl+C to stop all Docker services):"
echo ""
docker compose -f "${REPO_DIR}/docker/docker-compose.edge.yml" logs --follow \
  --no-color rtsp_server prometheus latency_exporter
