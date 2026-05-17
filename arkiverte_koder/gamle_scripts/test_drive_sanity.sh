#!/usr/bin/env bash
# Lavnivå drivlinje-sjekk (Gazebo + diff_drive + relay). Nav2 skal være av.
#
# Forutsetning (annen terminal):
#   source install/setup.bash
#   ros2 launch moonmapper_bringup sim_rover_clean.launch.py
#
# Forventet (ROS-konvensjon):
#   A) linear.x>0, angular.z=0  → fremover i Gazebo, /odom twist.linear.x > 0, yaw stabilt
#   B) linear.x=0, angular.z>0 → kropp roterer CCW på stedet, /odom twist.angular.z > 0
#   C) begge >0                 → bue fremover mot venstre
#
# Debug (egne terminaler mens test kjører):
#   ros2 topic echo /cmd_vel --once
#   ros2 topic echo /diff_drive_controller/cmd_vel --once
#   ros2 topic echo /odom --once
#   ros2 topic echo /joint_states --once
#   ros2 run tf2_ros tf2_echo odom base_footprint
#
set -euo pipefail
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 ikke i PATH — source /opt/ros/jazzy/setup.bash og install/setup.bash først." >&2
  exit 1
fi

echo "=== A: fremover 3 s (linear.x=0.10, angular.z=0) ==="
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}" || true
sleep 1

echo "=== B: ren yaw 3 s (linear.x=0, angular.z=0.5) ==="
timeout 3 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" || true
sleep 1

echo "=== C: bue 4 s (linear.x=0.10, angular.z=0.3) ==="
timeout 4 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.3}}" || true

echo "=== Ferdig. Stopp med Ctrl+C i Gazebo-terminal om nødvendig. ==="
