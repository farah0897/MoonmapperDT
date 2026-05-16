#!/usr/bin/env bash
# Run while autonomous_exploration_full.launch.py is active (separate terminal).
set -euo pipefail

echo "=== MoonMapper autonomous exploration stack check ==="
if ! command -v ros2 &>/dev/null; then
  echo "ERROR: ros2 not in PATH. Run: source install/setup.bash" >&2
  exit 1
fi

if ! ros2 node list 2>/dev/null | grep -q .; then
  echo "ERROR: No ROS 2 nodes visible. Start launch in another terminal first:" >&2
  echo "  ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py use_sim_time:=true ..." >&2
  exit 1
fi

echo
echo "--- key nodes ---"
for n in controller_server planner_server bt_navigator frontier_explorer; do
  if ros2 node list 2>/dev/null | grep -qE "/${n}$|/${n} "; then
    echo "  OK  /${n}"
  else
    echo "  --  /${n} (not running — Nav2 ~12s delay, explorer ~+5s)"
  fi
done

echo
echo "--- Nav2 lifecycle ---"
for n in controller_server planner_server bt_navigator; do
  if out=$(timeout 3 ros2 lifecycle get "/${n}" 2>/dev/null); then
    printf "  /%-20s %s\n" "$n" "$(echo "$out" | tr '\n' ' ')"
  else
    echo "  /${n} — unavailable"
  fi
done

echo
echo "--- frontier_explorer topics ---"
for t in /frontier_explorer/status /frontier_explorer/current_goal; do
  if ros2 topic info "$t" 2>/dev/null | head -1 | grep -q "Type:"; then
    echo "  ${t}:"
    ros2 topic info "$t" 2>/dev/null | sed 's/^/    /'
  else
    echo "  ${t} — not advertised (wait for start_explorer delay)"
  fi
done

echo
echo "--- NavigateToPose action ---"
if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
  echo "  OK  /navigate_to_pose"
else
  echo "  --  /navigate_to_pose (missing — frontier cannot drive; wait Nav2 @12s or check launch log)"
fi

echo
echo "--- frontier status (once) ---"
timeout 4 ros2 topic echo /frontier_explorer/status --once 2>/dev/null | head -8 \
  || echo "  (no status yet)"

echo
echo "=== done ==="
