#!/usr/bin/env bash
# RTAB-Map + real-robot Nav2 diagnostics (run while stack is up).
set -euo pipefail

echo "=== MoonMapper RTAB-Map navigation diagnose ==="
if ! command -v ros2 &>/dev/null; then
  echo "ERROR: source install/setup.bash" >&2
  exit 1
fi

check_topic() {
  local t=$1
  echo
  echo "--- $t (once, 6s timeout) ---"
  timeout 6 ros2 topic echo "$t" --once 2>/dev/null | head -12 || echo "  WARN: no message on $t"
}

check_tf() {
  local a=$1 b=$2
  echo
  echo "--- tf2_echo $a $b (2s) ---"
  timeout 2 ros2 run tf2_ros tf2_echo "$a" "$b" 2>/dev/null | head -10 || echo "  MISSING"
}

check_topic /odom
check_tf odom base_link
check_tf map odom
check_tf map base_link
check_topic /rtabmap/map
check_topic /map
check_topic /scan
check_topic /cmd_vel

echo
echo "--- lifecycle (Nav2) ---"
for n in controller_server planner_server bt_navigator; do
  if out=$(timeout 2 ros2 lifecycle get "/$n" 2>/dev/null); then
    printf "  /%-22s %s\n" "$n" "$(echo "$out" | tr '\n' ' ')"
  else
    echo "  /$n (unavailable)"
  fi
done

echo
echo "Expect: NO static map->odom identity from nav2_static_map; map->odom from RTAB-Map only."
echo "Expect: /map relayed from /rtabmap/map; robot_base_frame=base_link in Nav2 params."
echo "Done."
