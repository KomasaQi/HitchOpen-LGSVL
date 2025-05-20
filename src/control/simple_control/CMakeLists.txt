cmake_minimum_required(VERSION 3.8)
project(simple_control)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(
  include
)


set(${PROJECT_NAME}_SRC
  src/controller_manager_node.cpp
)


set(${PROJECT_NAME}_PLUGIN_SRC
  src/plugins/longitudinal/pid_controller.cpp
  src/plugins/lateral/pure_pursuit.cpp
)

# Create the library for plugins
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_PLUGIN_SRC}
)

# Create the executable node
ament_auto_add_executable(controller_manager_node
  ${${PROJECT_NAME}_SRC}
)

# Link the executable with the library
target_link_libraries(controller_manager_node
  ${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  params
  include
)
