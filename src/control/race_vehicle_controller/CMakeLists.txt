cmake_minimum_required(VERSION 3.8)
project(race_vehicle_controller)

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(CGAL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_SRC
  src/rvc_node.cpp
  src/pid_controller.cpp
  src/rvc_utils.cpp
)

set(${PROJECT_NAME}_PLUGIN_SRC
  src/plugins/compliance.cpp
  src/plugins/lon_control.cpp
  src/plugins/override.cpp
  src/plugins/pure_pursuit.cpp
  src/plugins/timeout.cpp
  src/plugins/input_validation.cpp
  src/plugins/lateral_compenstation.cpp
  src/plugins/gear_manager.cpp
  src/plugins/model_based_lon_control.cpp
  src/plugins/model_based_lat_control.cpp
  src/plugins/path_planner.cpp
  src/plugins/abs.cpp
  src/plugins/cbf_lane_keeping.cpp
  src/plugins/add_steer_bias.cpp
  src/plugins/lqg_controller_plugin.cpp
  src/plugins/test_harness.cpp
  src/plugins/check_outside_track.cpp
  src/plugins/art_mpc_plugin.cpp
  src/plugins/compute_slip_angle.cpp
  src/plugins/synthetic_steering_command.cpp
  src/plugins/step_steering_command.cpp
  src/plugins/push2pass_handler.cpp
)

# Create this project as an executable.
ament_auto_add_executable(${PROJECT_NAME}_node_exe
  ${${PROJECT_NAME}_SRC}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(TEST_SOURCES test/test_rvc.cpp)
  set(TEST_RVC_EXE test_rvc)
  ament_add_gtest(${TEST_RVC_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_RVC_EXE} ${PROJECT_NAME})
endif()

# Plugins
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_PLUGIN_SRC}
  ${${PROJECT_NAME}_SRC}
)

target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
target_link_libraries(${PROJECT_NAME}_node_exe CGAL::CGAL)

pluginlib_export_plugin_description_file(race_vehicle_controller plugins.xml)

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
  include
)
