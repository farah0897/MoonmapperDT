#!/usr/bin/env bash
# Nav2 + map sanity check for MoonMapper baseline (run while sim + Nav2 are up).
set -euo pipefail

echo "=== MoonMapper Nav2 health check ==="
if ! command -v ros2 &>/dev/null; then
  echo "ERROR: ros2 not in PATH. Source ROS + workspace: source install/setup.bash" >&2
  exit 1
fi

echo
echo "--- /map (nav_msgs/OccupancyGrid) ---"
timeout 6 ros2 topic echo /map --once 2>/dev/null | python3 <<'PY'
import math, sys, yaml
raw = sys.stdin.read()
if "does not appear to be published" in raw:
    print("  WARN: /map not published (start map_server / Nav2 bringup)")
    sys.exit(0)
try:
    d = yaml.safe_load(raw)
except Exception as e:
    print("  WARN: could not parse /map message:", e)
    sys.exit(0)
info = d.get("info") or {}
w = int(info.get("width", 0))
h = int(info.get("height", 0))
res = float(info.get("resolution", 0.0))
origin = info.get("origin") or {}
pos = origin.get("position") or {}
ox = float(pos.get("x", 0.0))
oy = float(pos.get("y", 0.0))
ori = origin.get("orientation") or {}
qx = float(ori.get("x", 0.0))
qy = float(ori.get("y", 0.0))
qz = float(ori.get("z", 0.0))
qw = float(ori.get("w", 1.0))
# yaw about z from quaternion
siny_cosp = 2.0 * (qw * qz + qx * qy)
cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
yaw = math.atan2(siny_cosp, cosy_cosp)
corners = []
for i in (0, w):
    for j in (0, h):
        mx, my = i * res, j * res
        wx = ox + mx * math.cos(yaw) - my * math.sin(yaw)
        wy = oy + mx * math.sin(yaw) + my * math.cos(yaw)
        corners.append((wx, wy))
xs = [c[0] for c in corners]
ys = [c[1] for c in corners]
print(f"  width={w} height={h} resolution={res}")
print(f"  origin position=({ox:.4f}, {oy:.4f}) yaw={yaw:.4f} rad")
print(f"  axis-aligned bounds x=[{min(xs):.3f}, {max(xs):.3f}] y=[{min(ys):.3f}, {max(ys):.3f}]")
open("/tmp/mm_nav2_map_bounds.txt", "w").write(f"{min(xs)} {max(xs)} {min(ys)} {max(ys)}\n")
PY

echo
echo "--- tf2_echo map base_footprint (2s) ---"
timeout 2 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | head -18 || echo "  WARN: no transform map->base_footprint"

echo
echo "--- /global_costmap/costmap (first lines) ---"
timeout 4 ros2 topic echo /global_costmap/costmap --once 2>/dev/null | head -22 || echo "  WARN: no global costmap"

echo
echo "--- /local_costmap/costmap (first lines) ---"
timeout 4 ros2 topic echo /local_costmap/costmap --once 2>/dev/null | head -22 || echo "  WARN: no local costmap"

echo
echo "--- lifecycle states ---"
for n in map_server amcl planner_server controller_server bt_navigator; do
  if out=$(timeout 2 ros2 lifecycle get "/$n" 2>/dev/null); then
    printf "  /%-20s %s\n" "$n" "$(echo "$out" | tr '\n' ' ')"
  else
    echo "  /$n (unavailable)"
  fi
done

echo
echo "--- cmd_vel chain ---"
for t in /cmd_vel_smoothed /cmd_vel_raw /cmd_vel; do
  echo "  $t:"
  timeout 2 ros2 topic echo "$t" --once 2>/dev/null | head -6 || echo "    (no data)"
done

echo
echo "--- robot inside /map AABB? ---"
if [[ -f /tmp/mm_nav2_map_bounds.txt ]]; then
  read -r bx0 bx1 by0 by1 < /tmp/mm_nav2_map_bounds.txt
  tfout=$(timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null || true)
  tx=$(echo "$tfout" | sed -n 's/.*Translation: *\[\([^,]*\), *\([^,]*\),.*/\1/p' | head -1)
  ty=$(echo "$tfout" | sed -n 's/.*Translation: *\[\([^,]*\), *\([^,]*\),.*/\2/p' | head -1)
  if [[ -n "${tx:-}" && -n "${ty:-}" ]]; then
    python3 - <<PY
bx0, bx1, by0, by1 = float("$bx0"), float("$bx1"), float("$by0"), float("$by1")
tx, ty = float("$tx"), float("$ty")
m = 0.2
ok = (bx0 + m <= tx <= bx1 - m) and (by0 + m <= ty <= by1 - m)
print(f"  map AABB (inner margin {m}m): x[{bx0:.3f},{bx1:.3f}] y[{by0:.3f},{by1:.3f}]")
print(f"  base_footprint: ({tx:.3f}, {ty:.3f})")
print("  STATUS:", "OK" if ok else "WARN: robot near/outside map bounds — use padded map or 2D Pose Estimate")
PY
  else
    echo "  (could not parse Translation from tf2_echo)"
  fi
else
  echo "  (no /map bounds; skip)"
fi

echo
echo "NOTE: If planner_server dies (e.g. exit -11), stop entire Nav2 launch (Ctrl+C) and restart:"
echo "  ros2 launch moonmapper_nav2 nav2_bringup_test.launch.py ..."
echo "=== done ==="
