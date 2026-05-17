#!/usr/bin/env bash
# Static-map Nav2 localization diagnostic (run while sim + nav2_static_map are up).
set -euo pipefail

MODE="${1:-odom}"
echo "=== MoonMapper Nav2 localization diagnose (mode=${MODE}) ==="
if ! command -v ros2 &>/dev/null; then
  echo "ERROR: ros2 not in PATH. Source: source install/setup.bash" >&2
  exit 1
fi

echo
echo "--- lifecycle (localization + navigation) ---"
for n in map_server amcl lifecycle_manager_localization \
  planner_server controller_server smoother_server \
  behavior_server bt_navigator velocity_smoother collision_monitor; do
  if out=$(timeout 2 ros2 lifecycle get "/$n" 2>/dev/null); then
    printf "  /%-28s %s\n" "$n" "$(echo "$out" | tr '\n' ' ')"
  else
    echo "  /$n (not running)"
  fi
done

echo
echo "--- /map ---"
timeout 5 ros2 topic echo /map --once 2>/dev/null | head -20 || echo "  WARN: /map not available"

echo
echo "--- TF chains ---"
for pair in "map odom" "odom base_footprint" "map base_footprint"; do
  echo "  tf2_echo $pair (1.5s):"
  timeout 2 ros2 run tf2_ros tf2_echo $pair 2>/dev/null | head -8 || echo "    MISSING"
done

echo
echo "--- global_costmap frame ---"
timeout 4 ros2 topic echo /global_costmap/costmap --once 2>/dev/null | grep -E '^  frame_id:|^header:' | head -4 \
  || echo "  WARN: no global costmap"

echo
echo "--- cmd_vel chain ---"
for t in /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_raw /cmd_vel; do
  hz=$(timeout 2 ros2 topic hz "$t" 2>/dev/null | tail -1 || true)
  if [[ -n "$hz" ]]; then
    echo "  $t: $hz"
  else
    echo "  $t: (no traffic or missing)"
  fi
done

echo
if [[ "$MODE" == "odom" ]]; then
  echo "--- odom mode checks ---"
  echo "  Expect: map_server ACTIVE, amcl absent, map->odom identity (0,0,0)"
  echo "  No RViz «2D Pose Estimate» required."
elif [[ "$MODE" == "amcl" ]]; then
  echo "--- amcl mode checks ---"
  echo "  Expect: map_server + amcl ACTIVE, map->odom from AMCL (non-identity after converge)"
  echo "  Match launch initial_x/y/yaw to Gazebo spawn."
else
  echo "  Usage: $0 [odom|amcl]"
fi
echo
echo "Done."
