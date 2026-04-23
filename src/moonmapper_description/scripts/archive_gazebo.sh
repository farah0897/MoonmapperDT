#!/usr/bin/env bash
# archive_gazebo.sh
#
# Flytter Gazebo-filene inn i _archive/gazebo/ igjen og gjenoppretter
# package.xml + CMakeLists.txt til Unity-oppsettet.
#
# DEPRECATED (2026):
#   This script is **destructive** by default (it moves source files and overwrites build files),
#   which makes the workspace stateful and easy to break for student teams.
#   Prefer keeping Gazebo + Unity assets in-tree and selecting the desired launch stack.
#   If you still need the old behavior, run with:
#     FORCE_DESTRUCTIVE=1 bash .../archive_gazebo.sh
#
# Bruk:
#     bash src/moonmapper_description/scripts/archive_gazebo.sh
#     colcon build --packages-select moonmapper_description --symlink-install
#     source install/setup.bash
#     ros2 launch moonmapper_description unity_minimal.launch.py

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCHIVE="$PKG_DIR/_archive/gazebo"

mkdir -p "$ARCHIVE/urdf" "$ARCHIVE/launch" "$ARCHIVE/config"

echo "== Arkiverer Gazebo-filer =="

if [[ "${FORCE_DESTRUCTIVE:-0}" != "1" ]]; then
  echo "INFO: Non-destructive mode (default). No files will be moved or overwritten."
  echo "      Set FORCE_DESTRUCTIVE=1 to run the original destructive workflow."
  exit 0
fi

[[ -f "$PKG_DIR/urdf/moonmapper_rover_gazebo.urdf.xacro" ]] && \
    mv "$PKG_DIR/urdf/moonmapper_rover_gazebo.urdf.xacro" "$ARCHIVE/urdf/"

[[ -f "$PKG_DIR/launch/gazebo_rover.launch.py" ]] && \
    mv "$PKG_DIR/launch/gazebo_rover.launch.py" "$ARCHIVE/launch/"

[[ -f "$PKG_DIR/config/ros_gz_bridge.yaml" ]] && \
    mv "$PKG_DIR/config/ros_gz_bridge.yaml" "$ARCHIVE/config/"

[[ -f "$PKG_DIR/config/wheel_controllers.yaml" ]] && \
    mv "$PKG_DIR/config/wheel_controllers.yaml" "$ARCHIVE/config/"

[[ -f "$PKG_DIR/Gazebo_test.md" ]] && \
    mv "$PKG_DIR/Gazebo_test.md" "$ARCHIVE/"

[[ -d "$PKG_DIR/worlds" ]]     && mv "$PKG_DIR/worlds"     "$ARCHIVE/"
[[ -d "$PKG_DIR/gz_plugins" ]] && mv "$PKG_DIR/gz_plugins" "$ARCHIVE/"
[[ -d "$PKG_DIR/hooks" ]]      && mv "$PKG_DIR/hooks"      "$ARCHIVE/"

# --- Unity-variant av package.xml ---------------------------------------
cat > "$PKG_DIR/package.xml" <<'XML'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>moonmapper_description</name>
  <version>0.3.0</version>
  <description>
    URDF/Xacro, meshes, RViz config og Unity-integrasjon for MoonMapper
    rocker-bogie-konsept. Simuleringen kjoerer i Unity via
    ROS-TCP-Connector/Endpoint. Gazebo-oppsettet er arkivert i _archive/gazebo/.
  </description>
  <maintainer email="user@example.com">MoonMapper Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ament_index_python</exec_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <exec_depend>ros_tcp_endpoint</exec_depend>
  <exec_depend>teleop_twist_keyboard</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
XML

# --- Unity-variant av CMakeLists.txt ------------------------------------
cat > "$PKG_DIR/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(moonmapper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/rocker_diff_joint.py
  launch/unity_cmd_vel_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
CMAKE

echo
echo "== FERDIG =="
echo "Neste steg:"
echo "  colcon build --packages-select moonmapper_description --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch moonmapper_description unity_minimal.launch.py"
