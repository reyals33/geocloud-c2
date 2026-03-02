#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════
#  run_console.sh — GeoCloud Fleet Operator Console Launcher
#
#  Starts the WebSocket server that bridges:
#    /tb3_2/camera/image_raw  →  ws://localhost:8765  →  browser
#    browser cmd_vel JSON     →  /tb3_2/cmd_vel       →  robot
#
#  Usage:
#    ./scripts/run_console.sh           # normal
#    ./scripts/run_console.sh --check   # check deps only
# ════════════════════════════════════════════════════════════════
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${REPO}/.venv"
CHECK_ONLY="${1:-}"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
ok()   { echo -e "${GREEN}  [OK]${NC}  $*"; }
warn() { echo -e "${YELLOW}  [WARN]${NC} $*"; }
fail() { echo -e "${RED}  [FAIL]${NC} $*"; FAIL=1; }

echo ""
echo "══════════════════════════════════════════"
echo "  GeoCloud Fleet Console — Pre-flight"
echo "══════════════════════════════════════════"
FAIL=0

# ── 1. ROS2 ──────────────────────────────────────────────────
if [ -z "${ROS_DISTRO:-}" ]; then
  for setup in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [ -f "$setup" ]; then
      # shellcheck disable=SC1090
      source "$setup"
      warn "ROS2 not sourced — sourced ${setup} for this session"
      break
    fi
  done
fi
[ -n "${ROS_DISTRO:-}" ] && ok "ROS2 ${ROS_DISTRO}" || fail "No ROS2 found"

# ── 2. Workspace overlay (optional — has multirobot_map_merge) ──
if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/ros2_ws/install/setup.bash"
  ok "ros2_ws overlay sourced"
fi

# ── 3. Venv ──────────────────────────────────────────────────
if [ -f "${VENV}/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "${VENV}/bin/activate"
  ok "venv: ${VENV}"
else
  fail "venv not found — run: python3 -m venv --system-site-packages ${VENV} && ${VENV}/bin/pip install -r ${REPO}/requirements.txt"
  exit 1
fi

# ── 4. Python dependencies ────────────────────────────────────
"${VENV}/bin/python3" -c "import rclpy" 2>/dev/null \
  && ok "rclpy (via system-site-packages)" \
  || fail "rclpy not found — ensure --system-site-packages venv and ROS2 is sourced"

"${VENV}/bin/python3" -c "import websockets" 2>/dev/null \
  && ok "websockets" \
  || { fail "websockets missing → ${VENV}/bin/pip install 'websockets>=12.0'"; FAIL=1; }

"${VENV}/bin/python3" -c "import cv2" 2>/dev/null \
  && ok "cv2 (opencv)" \
  || warn "cv2 not found — video encoding disabled. Install: ${VENV}/bin/pip install opencv-python-headless"

"${VENV}/bin/python3" -c "import numpy" 2>/dev/null \
  && ok "numpy" \
  || fail "numpy missing → ${VENV}/bin/pip install numpy"

# ── 5. ROS_DOMAIN_ID ─────────────────────────────────────────
export ROS_DOMAIN_ID=42
ok "ROS_DOMAIN_ID=42"

# ── Summary ───────────────────────────────────────────────────
echo ""
if [ "${FAIL}" -ne 0 ]; then
  echo -e "${RED}  PRE-FLIGHT FAILED — fix items above.${NC}"
  exit 1
fi
echo -e "${GREEN}  PRE-FLIGHT PASSED${NC}"
echo ""

[ "${CHECK_ONLY}" = "--check" ] && exit 0

# ── Launch ────────────────────────────────────────────────────
echo "══════════════════════════════════════════"
echo "  Starting console server..."
echo "  Open in browser:"
echo "    file://${REPO}/web/console.html"
echo ""
echo "  Or if running remotely, copy console.html"
echo "  and change WS URL to ws://<server-ip>:8765"
echo "══════════════════════════════════════════"
echo ""

exec "${VENV}/bin/python3" "${REPO}/scripts/videg_console_server.py"
