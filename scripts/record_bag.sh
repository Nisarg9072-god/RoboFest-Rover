#!/usr/bin/env bash
# =============================================================================
# record_bag.sh
# ROBOFEST Rover — ROS 2 bag recording script
# =============================================================================
# Records ALL topics to a timestamped bag file in logs/
#
# Usage:
#   bash scripts/record_bag.sh            # record all topics
#   bash scripts/record_bag.sh --nav      # record navigation topics only
#   bash scripts/record_bag.sh --sensors  # record sensor topics only
# =============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$WORKSPACE_DIR/logs"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="rover_run_$TIMESTAMP"

# Source workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "=============================================="
echo " ROBOFEST Rover — ROS 2 Bag Recording"
echo " Output: $LOG_DIR/$BAG_NAME"
echo "=============================================="
echo " Press Ctrl+C to stop recording."
echo "=============================================="

if [[ "$1" == "--nav" ]]; then
  # Navigation topics only
  ros2 bag record \
    -o "$LOG_DIR/$BAG_NAME" \
    /scan \
    /odom \
    /odometry/filtered \
    /localization/pose \
    /map \
    /path \
    /cmd_vel \
    /cmd_vel_safe \
    /goal_pose \
    /emergency_stop \
    /mission/state

elif [[ "$1" == "--sensors" ]]; then
  # Sensor topics only
  ros2 bag record \
    -o "$LOG_DIR/$BAG_NAME" \
    /scan \
    /camera/image_raw \
    /imu/data \
    /tof/front_left \
    /tof/front_center \
    /tof/front_right \
    /ultrasonic/front \
    /ultrasonic/rear \
    /battery/status

else
  # Record everything
  ros2 bag record \
    -o "$LOG_DIR/$BAG_NAME" \
    -a
fi

echo ""
echo "[record] Bag saved to: $LOG_DIR/$BAG_NAME"
