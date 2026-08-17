#!/usr/bin/env bash
# =============================================================================
# test_motors.sh
# ROBOFEST Rover — Motor bring-up test script (Testing Level 1–4)
#
# Publishes simple velocity commands directly to /cmd_vel_safe
# to verify each motor direction and response.
#
# Usage:
#   bash scripts/test_motors.sh
#
# Pre-conditions:
#   - Arduino connected and firmware running
#   - arduino_interface_node running
#   - safety_controller_node running (or test directly on /cmd_vel_safe)
#
# WARNING:
#   This script bypasses the safety controller.
#   Keep the rover on a stand with wheels off the ground during initial tests.
#   Have the hardware E-Stop button accessible at all times.
# =============================================================================

set -e

echo "=============================================="
echo " ROBOFEST Rover — Motor Bring-Up Test"
echo " WARNING: Wheels may spin. Use a stand."
echo " Press Ctrl+C at any time. E-Stop accessible."
echo "=============================================="
read -p "Press Enter to begin, or Ctrl+C to abort..."

# Source workspace
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  source "$WORKSPACE_DIR/install/setup.bash"
fi

_pub_cmd() {
  local linear=$1
  local angular=$2
  local duration=$3
  echo "  → linear=$linear  angular=$angular  duration=${duration}s"
  timeout "$duration" ros2 topic pub --once /cmd_vel_safe \
    geometry_msgs/msg/Twist \
    "{linear: {x: $linear, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $angular}}" \
    || true
  sleep 0.5
}

_stop() {
  ros2 topic pub --once /cmd_vel_safe \
    geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    > /dev/null 2>&1 || true
}

echo ""
echo "[TEST 1] Forward — all wheels forward (0.2 m/s, 2 s)"
_pub_cmd 0.2 0.0 2
_stop; sleep 1

echo "[TEST 2] Reverse — all wheels reverse (-0.2 m/s, 2 s)"
_pub_cmd -0.2 0.0 2
_stop; sleep 1

echo "[TEST 3] Turn left — angular +0.5 rad/s, 2 s"
_pub_cmd 0.1 0.5 2
_stop; sleep 1

echo "[TEST 4] Turn right — angular -0.5 rad/s, 2 s"
_pub_cmd 0.1 -0.5 2
_stop; sleep 1

echo "[TEST 5] Rotate in place CCW — linear 0, angular +0.5, 2 s"
_pub_cmd 0.0 0.5 2
_stop; sleep 1

echo "[TEST 6] Rotate in place CW — linear 0, angular -0.5, 2 s"
_pub_cmd 0.0 -0.5 2
_stop; sleep 1

echo ""
echo "=============================================="
echo " Motor test complete."
echo " Check:"
echo "   1. All 6 wheels spun in expected directions."
echo "   2. No unusual sounds (binding, grinding)."
echo "   3. ros2 topic echo /odom shows non-zero velocities."
echo "   4. Check motor driver LEDs / heat after test."
echo "=============================================="
