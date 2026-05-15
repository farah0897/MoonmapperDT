cmake_minimum_required(VERSION 3.16)
project(moonmapper_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
  scripts/nav2_health_check.sh
  scripts/nav2_mission_health_check.sh
  scripts/nav2_localization_diagnose.sh
  scripts/nav2_mission_client_node
  scripts/nav2_tf_chain_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
