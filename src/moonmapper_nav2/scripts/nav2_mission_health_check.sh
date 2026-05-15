#!/usr/bin/env bash
# Nav2 mission stack sanity (run while Gazebo + nav2_coverage_mission_test are up).
set -euo pipefail

echo "=== MoonMapper Nav2 mission health check ==="
if ! command -v ros2 &>/dev/null; then
  echo "ERROR: ros2 not in PATH. Source ROS + workspace: source install/setup.bash" >&2
  exit 1
fi

echo
echo "--- ros2 action list | grep -i navigate ---"
ros2 action list 2>/dev/null | grep -i navigate || echo "  (no navigate* actions)"

echo
echo "--- ros2 action info /navigate_through_poses ---"
timeout 4 ros2 action info /navigate_through_poses 2>/dev/null || echo "  WARN: /navigate_through_poses unavailable"

echo
echo "--- lifecycle (map_server, amcl, planner, controller, bt_navigator) ---"
for n in map_server amcl planner_server controller_server bt_navigator; do
  if out=$(timeout 2 ros2 lifecycle get "/$n" 2>/dev/null); then
    printf "  /%-22s %s\n" "$n" "$(echo "$out" | tr '\n' ' ')"
  else
    echo "  /$n (unavailable)"
  fi
done

echo
echo "--- /plan (once) ---"
timeout 6 ros2 topic echo /plan --once 2>/dev/null | head -12 || echo "  WARN: no /plan"

echo
echo "--- /cmd_vel (once) ---"
timeout 4 ros2 topic echo /cmd_vel --once 2>/dev/null | head -8 || echo "  WARN: no /cmd_vel"

echo
echo "--- /cmd_vel_nav2 (once, optional) ---"
if timeout 2 ros2 topic type /cmd_vel_nav2 &>/dev/null; then
  timeout 4 ros2 topic echo /cmd_vel_nav2 --once 2>/dev/null | head -8 || echo "  (no data yet)"
else
  echo "  (topic /cmd_vel_nav2 not present — OK for this stack)"
fi

echo
echo "--- /cmd_vel_raw (once) ---"
timeout 4 ros2 topic echo /cmd_vel_raw --once 2>/dev/null | head -8 || echo "  WARN: no /cmd_vel_raw"

echo
echo "--- /global_costmap/costmap (once, truncated) ---"
timeout 5 ros2 topic echo /global_costmap/costmap --once 2>/dev/null | head -18 || echo "  WARN: no global costmap"

echo
echo "--- /local_costmap/costmap (once, truncated) ---"
timeout 5 ros2 topic echo /local_costmap/costmap --once 2>/dev/null | head -18 || echo "  WARN: no local costmap"

echo
echo "--- tf2_echo map base_footprint (3s) ---"
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | head -22 || echo "  WARN: no map->base_footprint (set 2D Pose Estimate?)"

echo
echo "=== done ==="
