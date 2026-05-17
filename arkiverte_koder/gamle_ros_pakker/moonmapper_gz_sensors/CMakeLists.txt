cmake_minimum_required(VERSION 3.8)
project(moonmapper_gz_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moonmapper_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(moonmapper_triad_spectroscopy SHARED
  gz_plugins/src/TriadSpectroscopy.cc
)

target_link_libraries(moonmapper_triad_spectroscopy
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  yaml-cpp
)

ament_target_dependencies(moonmapper_triad_spectroscopy
  rclcpp
  moonmapper_msgs
)

install(TARGETS moonmapper_triad_spectroscopy
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

