#!/usr/bin/env bash
# =============================================================================
# build.sh
# ROBOFEST Rover — workspace build script
# =============================================================================
# Usage:
#   bash scripts/build.sh           # full build
#   bash scripts/build.sh --clean   # clean build (removes install/, build/, log/)
# =============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_DIR"

echo "=============================================="
echo " ROBOFEST Rover — ROS 2 Workspace Build"
echo " Workspace: $WORKSPACE_DIR"
echo "=============================================="

# Source ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
  # Attempt to source Humble, then Iron
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[build] Sourced ROS 2 Humble"
  elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
    echo "[build] Sourced ROS 2 Iron"
  else
    echo "[build] ERROR: No ROS 2 installation found at /opt/ros/"
    echo "       Install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
  fi
fi

# Optional clean build
if [[ "$1" == "--clean" ]]; then
  echo "[build] Cleaning build/, install/, log/ ..."
  rm -rf build/ install/ log/
fi

# Build
echo "[build] Running colcon build ..."
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tee logs/build_$(date +%Y%m%d_%H%M%S).log

echo ""
echo "[build] Build complete."
echo "[build] Source the workspace: source install/setup.bash"
echo "=============================================="
