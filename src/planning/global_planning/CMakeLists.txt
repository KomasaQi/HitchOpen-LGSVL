cmake_minimum_required(VERSION 3.8)
project(global_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(autoware_planning_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  autoware_planning_msgs
  GeographicLib
  visualization_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  lgsvl_msgs
)


ament_auto_add_executable(global_planning_node src/global_planning_node.cpp)
ament_auto_add_executable(gps_to_enu_converter src/gps_to_enu_converter.cpp)
ament_auto_add_executable(enu_logger src/enu_logger.cpp)

ament_target_dependencies(global_planning_node ${dependencies})
ament_target_dependencies(gps_to_enu_converter ${dependencies})
ament_target_dependencies(enu_logger ${dependencies})

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_auto_package()