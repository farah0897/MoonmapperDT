#!/usr/bin/env bash
# restore_gazebo.sh
#
# Flytter Gazebo-filene tilbake fra _archive/gazebo/ og gjenoppretter
# package.xml + CMakeLists.txt slik at `colcon build` bygger Gazebo-
# stacken igjen.
#
# DEPRECATED (2026):
#   This script is **destructive** by default (moves files and overwrites build files).
#   It is kept only for legacy workflows. Prefer keeping Gazebo + Unity assets in-tree.
#   If you still need the old behavior, run with:
#     FORCE_DESTRUCTIVE=1 bash .../restore_gazebo.sh
#
# Bruk:
#     bash src/moonmapper_description/scripts/restore_gazebo.sh
#     colcon build --packages-select moonmapper_description --symlink-install
#     source install/setup.bash
#     ros2 launch moonmapper_description gazebo_rover.launch.py
#
# For aa gaa tilbake til Unity-oppsettet: bruk archive_gazebo.sh.

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCHIVE="$PKG_DIR/_archive/gazebo"

if [[ ! -d "$ARCHIVE" ]]; then
    echo "FEIL: fant ikke $ARCHIVE" >&2
    exit 1
fi

echo "== Flytter Gazebo-filer tilbake =="

if [[ "${FORCE_DESTRUCTIVE:-0}" != "1" ]]; then
    echo "INFO: Non-destructive mode (default). No files will be moved or overwritten."
    echo "      Set FORCE_DESTRUCTIVE=1 to run the original destructive workflow."
    exit 0
fi

# Filer
[[ -f "$ARCHIVE/urdf/moonmapper_rover_gazebo.urdf.xacro" ]] && \
    mv "$ARCHIVE/urdf/moonmapper_rover_gazebo.urdf.xacro" "$PKG_DIR/urdf/"

[[ -f "$ARCHIVE/launch/gazebo_rover.launch.py" ]] && \
    mv "$ARCHIVE/launch/gazebo_rover.launch.py" "$PKG_DIR/launch/"

[[ -f "$ARCHIVE/config/ros_gz_bridge.yaml" ]] && \
    mv "$ARCHIVE/config/ros_gz_bridge.yaml" "$PKG_DIR/config/"

[[ -f "$ARCHIVE/config/wheel_controllers.yaml" ]] && \
    mv "$ARCHIVE/config/wheel_controllers.yaml" "$PKG_DIR/config/"

[[ -f "$ARCHIVE/Gazebo_test.md" ]] && \
    mv "$ARCHIVE/Gazebo_test.md" "$PKG_DIR/"

# Mapper
[[ -d "$ARCHIVE/worlds" ]]     && mv "$ARCHIVE/worlds"     "$PKG_DIR/"
[[ -d "$ARCHIVE/gz_plugins" ]] && mv "$ARCHIVE/gz_plugins" "$PKG_DIR/"
[[ -d "$ARCHIVE/hooks" ]]      && mv "$ARCHIVE/hooks"      "$PKG_DIR/"

# Rydd tomme arkiv-underkataloger
rmdir "$ARCHIVE/urdf" "$ARCHIVE/launch" "$ARCHIVE/config" 2>/dev/null || true

# --- Patch package.xml ---------------------------------------------------
cat > "$PKG_DIR/package.xml" <<'XML'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>moonmapper_description</name>
  <version>0.3.0</version>
  <description>
    URDF/Xacro, meshes, RViz config, Gazebo Sim 8 plugin og Unity-
    integrasjon for MoonMapper rocker-bogie-konsept.
    Gazebo-stack AKTIV (restore_gazebo.sh). Bruk archive_gazebo.sh
    for aa slaa over til ren Unity-stack.
  </description>
  <maintainer email="user@example.com">MoonMapper Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Gazebo Sim 8 plugin build -->
  <build_depend>libgz-sim8-dev</build_depend>
  <build_depend>libgz-plugin2-dev</build_depend>

  <!-- RViz / beskrivelse -->
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

  <!-- Gazebo Sim 8 integrasjon -->
  <exec_depend>libgz-sim8</exec_depend>
  <exec_depend>libgz-plugin2</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>

  <!-- ros2_control stack -->
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>

  <!-- Unity-stack (valgfri, men holdt inne saa launch-filene ikke knekker) -->
  <exec_depend>ros_tcp_endpoint</exec_depend>
  <exec_depend>teleop_twist_keyboard</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
XML

# --- Patch CMakeLists.txt ------------------------------------------------
cat > "$PKG_DIR/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.8)
project(moonmapper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------
# Gazebo Sim 8 system plugin (optional)
# -----------------------------------------------------------------
find_package(gz-sim8 QUIET)
find_package(gz-plugin2 QUIET)

if(gz-sim8_FOUND AND gz-plugin2_FOUND)
  message(STATUS "Building moonmapper_rocker_bogie_differential plugin for gz-sim8")
  add_library(moonmapper_rocker_bogie_differential SHARED
    gz_plugins/src/RockerBogieDifferential.cc
  )
  target_link_libraries(moonmapper_rocker_bogie_differential
    PRIVATE gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
  install(TARGETS moonmapper_rocker_bogie_differential
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "gz-sim8 / gz-plugin2 not found; skipping Gazebo plugin build.")
endif()

install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/rocker_diff_joint.py
  launch/unity_cmd_vel_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
CMAKE

echo
echo "== FERDIG =="
echo "Neste steg:"
echo "  cd $(dirname "$PKG_DIR")/../../.."
echo "  colcon build --packages-select moonmapper_description --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch moonmapper_description gazebo_rover.launch.py"
