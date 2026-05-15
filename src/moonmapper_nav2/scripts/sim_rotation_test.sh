#!/usr/bin/env bash
# In-place rotation test: Gazebo physics, /odom yaw, RViz model must agree.
set -euo pipefail

RATE="${1:-10}"
ANG="${2:-0.7}"
DUR="${3:-8}"

echo "=== MoonMapper sim rotation test ==="
echo "Publishing angular.z=${ANG} rad/s on /cmd_vel for ~${DUR}s (${RATE} Hz)"
echo "Prerequisite: ros2 launch moonmapper_bringup sim_rover_clean.launch.py"
echo "Optional: stop Nav2 stack so safety does not gate /cmd_vel_raw"
echo

if ! command -v ros2 &>/dev/null; then
  echo "ERROR: source install/setup.bash first" >&2
  exit 1
fi

echo "--- Before (odom yaw) ---"
timeout 3 ros2 topic echo /odom --once 2>/dev/null | grep -E 'orientation:|angular:' | head -6 || true

echo
echo "--- Publishing ---"
timeout "${DUR}" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANG}}}" -r "${RATE}" &
PUB_PID=$!
sleep "${DUR}"
wait "${PUB_PID}" 2>/dev/null || true

echo
echo "--- After (odom yaw) ---"
timeout 3 ros2 topic echo /odom --once 2>/dev/null | grep -E 'orientation:|angular:' | head -6 || true

echo
echo "Run: ros2 run moonmapper_nav2 sim_drive_diagnose --ros-args -p use_sim_time:=true"
echo "Expect: opposite wheel speeds in Gazebo, body rotates, /odom yaw changes, RViz matches."
