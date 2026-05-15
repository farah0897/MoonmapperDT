cmake_minimum_required(VERSION 3.16)
project(moonmapper_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz maps DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/slam_health_check.sh
    scripts/save_current_map.sh
    scripts/pad_occupancy_map.py
  DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS moonmapper_mapping/waypoint_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_mission_node)
install(
  PROGRAMS moonmapper_mapping/coverage_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coverage_mission_node)

ament_package()
