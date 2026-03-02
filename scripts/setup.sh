#!/usr/bin/env bash
# setup.sh
# ─────────────────────────────────────────────────────────────────────
# GeoCloud Fleet Sim – One-Shot Dependency Installation
# Target: Ubuntu 22.04 + ROS2 Humble
#
# Usage:   bash setup.sh [--skip-px4] [--skip-vins] [--skip-build]
# Stages:
#   1. ROS2 Humble + Gazebo
#   2. Core ROS2 packages (Nav2, SLAM, MAVROS, image_transport, etc.)
#   3. PX4-Autopilot SITL (optional; large download)
#   4. VINS-Fusion ROS2 wrapper (optional; requires Ceres solver)
#   5. m-explore-ros2 (frontier exploration + map merge)
#   6. px4-offboard node
#   7. micro-XRCE-DDS agent
#   8. Docker (for edge broker stack)
#   9. Workspace build
# ─────────────────────────────────────────────────────────────────────

set -euo pipefail

# ── Flags ─────────────────────────────────────────────────────────────
SKIP_PX4=false
SKIP_VINS=false
SKIP_BUILD=false

for arg in "$@"; do
  case $arg in
    --skip-px4)   SKIP_PX4=true  ;;
    --skip-vins)  SKIP_VINS=true ;;
    --skip-build) SKIP_BUILD=true;;
  esac
done

WORKSPACE="${HOME}/ros2_ws"
echo "═══════════════════════════════════════════════════════"
echo " GeoCloud Fleet Sim Setup"
echo " Workspace: ${WORKSPACE}"
echo "═══════════════════════════════════════════════════════"

# ── Stage 1: ROS2 Humble ──────────────────────────────────────────────
echo ""
echo "── Stage 1: ROS2 Humble ────────────────────────────────"
if ! command -v ros2 &>/dev/null; then
  sudo apt update && sudo apt install -y software-properties-common curl
  sudo add-apt-repository universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
else
  echo "  ✓ ROS2 already installed"
fi
source /opt/ros/humble/setup.bash

# ── Stage 2: Core ROS2 packages ───────────────────────────────────────
echo ""
echo "── Stage 2: Core ROS2 packages ─────────────────────────"
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-ros-gz-bridge \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-ffmpeg-image-transport \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-topic-tools \
  ros-humble-micro-xrce-dds-agent \
  ros-humble-nav2-bringup \
  libgeographic-dev geographiclib-tools
# MAVROS geographiclib datasets
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh || true
echo "  ✓ Core packages installed"

# ── Stage 3: PX4 Autopilot SITL ───────────────────────────────────────
echo ""
echo "── Stage 3: PX4 Autopilot ──────────────────────────────"
if [ "$SKIP_PX4" = true ]; then
  echo "  ⚠ Skipped (--skip-px4)"
else
  if [ ! -d "${HOME}/PX4-Autopilot" ]; then
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive "${HOME}/PX4-Autopilot"
    cd "${HOME}/PX4-Autopilot"
    bash ./Tools/setup/ubuntu.sh --no-nuttx
    make px4_sitl gazebo-classic_iris
    cd -
  else
    echo "  ✓ PX4-Autopilot already present"
  fi
  # PX4 msgs for ROS2
  mkdir -p "${WORKSPACE}/src"
  if [ ! -d "${WORKSPACE}/src/px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git "${WORKSPACE}/src/px4_msgs" -b release/1.14
  fi
fi

# ── Stage 4: VINS-Fusion ──────────────────────────────────────────────
echo ""
echo "── Stage 4: VINS-Fusion ────────────────────────────────"
if [ "$SKIP_VINS" = true ]; then
  echo "  ⚠ Skipped (--skip-vins)"
else
  # Ceres Solver (VINS backend)
  sudo apt install -y libceres-dev libgoogle-glog-dev libatlas-base-dev
  mkdir -p "${WORKSPACE}/src"
  if [ ! -d "${WORKSPACE}/src/VINS-Fusion" ]; then
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git \
      "${WORKSPACE}/src/VINS-Fusion"
  fi
  echo "  ✓ VINS-Fusion cloned; will build in Stage 9"
fi

# ── Stage 5: m-explore-ros2 (exploration + map merge) ────────────────
echo ""
echo "── Stage 5: m-explore-ros2 ─────────────────────────────"
mkdir -p "${WORKSPACE}/src"
if [ ! -d "${WORKSPACE}/src/m-explore-ros2" ]; then
  git clone https://github.com/robo-friends/m-explore-ros2.git \
    "${WORKSPACE}/src/m-explore-ros2"
fi
echo "  ✓ m-explore-ros2 cloned"

# ── Stage 6: px4-offboard ─────────────────────────────────────────────
echo ""
echo "── Stage 6: px4-offboard ───────────────────────────────"
if [ ! -d "${WORKSPACE}/src/px4-offboard" ]; then
  git clone https://github.com/Jaeyoung-Lim/px4-offboard.git \
    "${WORKSPACE}/src/px4-offboard" || \
  echo "  ⚠ px4-offboard clone failed; install manually later"
fi

# ── Stage 7: Custom packages ──────────────────────────────────────────
echo ""
echo "── Stage 7: Custom packages ────────────────────────────"

# fleet_sim launch package
if [ ! -d "${WORKSPACE}/src/fleet_sim" ]; then
  mkdir -p "${WORKSPACE}/src/fleet_sim/launch"
  cp "$(dirname "$0")/../launch/fleet_sim.launch.py" \
    "${WORKSPACE}/src/fleet_sim/launch/" 2>/dev/null || true
  cat > "${WORKSPACE}/src/fleet_sim/package.xml" <<'PKG_XML'
<?xml version="1.0"?>
<package format="3">
  <name>fleet_sim</name>
  <version>2.0.0</version>
  <description>GeoCloud mixed fleet ROS2 simulation launch</description>
  <maintainer email="george@geocloudsolutions.com">George O'Connor</maintainer>
  <license>MIT</license>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>turtlebot3_gazebo</exec_depend>
  <exec_depend>turtlebot3_navigation2</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>explore_lite</exec_depend>
  <exec_depend>mavros</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>image_transport</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
PKG_XML

  cat > "${WORKSPACE}/src/fleet_sim/setup.py" <<'SETUP_PY'
from setuptools import setup
import os
from glob import glob
setup(
    name='fleet_sim',
    version='2.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fleet_sim']),
        ('share/fleet_sim', ['package.xml']),
        (os.path.join('share', 'fleet_sim', 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='George O Connor',
    maintainer_email='george@geocloudsolutions.com',
    description='GeoCloud fleet simulation',
    license='MIT',
    entry_points={},
)
SETUP_PY
  mkdir -p "${WORKSPACE}/src/fleet_sim/resource"
  touch "${WORKSPACE}/src/fleet_sim/resource/fleet_sim"
fi

# vins_swarm_fusion package
if [ ! -d "${WORKSPACE}/src/vins_swarm_fusion" ]; then
  ros2 pkg create --build-type ament_python vins_swarm_fusion \
    --destination-directory "${WORKSPACE}/src"
  cp "$(dirname "$0")/vins_swarm_fusion_node.py" \
    "${WORKSPACE}/src/vins_swarm_fusion/vins_swarm_fusion/" 2>/dev/null || true
fi

# ── Stage 8: Docker (edge broker) ────────────────────────────────────
echo ""
echo "── Stage 8: Docker ─────────────────────────────────────"
if ! command -v docker &>/dev/null; then
  curl -fsSL https://get.docker.com | sudo sh
  sudo usermod -aG docker "$USER"
  echo "  ✓ Docker installed – log out/in to use without sudo"
else
  echo "  ✓ Docker already installed"
fi

# ── Stage 9: Build workspace ──────────────────────────────────────────
echo ""
echo "── Stage 9: Build workspace ────────────────────────────"
if [ "$SKIP_BUILD" = true ]; then
  echo "  ⚠ Skipped (--skip-build)"
else
  cd "${WORKSPACE}"
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y || true
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  echo ""
  echo "  Add to ~/.bashrc:"
  echo "  source ${WORKSPACE}/install/setup.bash"
fi

echo ""
echo "═══════════════════════════════════════════════════════"
echo " ✓ Setup complete!"
echo ""
echo " Quick start:"
echo "   source ~/ros2_ws/install/setup.bash"
echo "   export TURTLEBOT3_MODEL=burger"
echo "   ros2 launch fleet_sim fleet_sim.launch.py"
echo ""
echo " Edge broker:"
echo "   cd <project_dir>/docker"
echo "   docker compose -f docker-compose.edge.yml up"
echo "═══════════════════════════════════════════════════════"
