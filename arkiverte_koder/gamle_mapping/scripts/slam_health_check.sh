#!/usr/bin/env bash
# SLAM health check (MoonMapper Steg 4). Kjør med: source install/setup.bash
set -euo pipefail

echo "=== /scan ==="
ros2 topic hz /scan --window 20 2>/dev/null | head -5 || true
ros2 topic echo /scan --once 2>/dev/null | head -30 || echo "(ingen /scan)"

echo ""
echo "=== /odom ==="
ros2 topic hz /odom --window 20 2>/dev/null | head -5 || true
ros2 topic echo /odom --once 2>/dev/null | head -25 || echo "(ingen /odom)"

echo ""
echo "=== slam_toolbox params ==="
ros2 param get /slam_toolbox scan_topic 2>/dev/null || echo "(ingen /slam_toolbox-node?)"
ros2 param get /slam_toolbox map_frame 2>/dev/null || true
ros2 param get /slam_toolbox odom_frame 2>/dev/null || true
ros2 param get /slam_toolbox base_frame 2>/dev/null || true

echo ""
echo "=== lifecycle state (forventet active etter launch) ==="
ros2 lifecycle get /slam_toolbox 2>/dev/null || echo "(ikke lifecycle eller node mangler)"

echo ""
echo "=== /map ==="
ros2 topic info /map -v 2>/dev/null | head -25 || echo "(ingen /map-publisher)"
timeout 3 ros2 topic echo /map --once 2>/dev/null | head -15 || echo "(timeout/ingen melding — sjekk at SLAM er aktiv)"

echo ""
echo "=== tf2 (Ctrl+C for aa avbryte hver) ==="
echo "odom -> base_footprint:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -8 || true

echo ""
echo "Ferdig."
