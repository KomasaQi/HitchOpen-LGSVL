cmake_minimum_required(VERSION 3.8)
project(autoware_lgsvl_interface)

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
  src/autoware_lgsvl_interface.cpp
)

# Create the executable node
ament_auto_add_executable(autoware_lgsvl_interface
  ${${PROJECT_NAME}_SRC}
)

# Create & install ament package.
ament_auto_package(INSTALL_TO_SHARE
  launch
  params
  include
)
