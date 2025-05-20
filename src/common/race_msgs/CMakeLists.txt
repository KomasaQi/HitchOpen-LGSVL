cmake_minimum_required(VERSION 3.5)
project(race_msgs)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StopType.msg"
  "msg/StrategyType.msg"
  "msg/BrakeReport.msg"
  "msg/CtManagerDiag.msg"
  "msg/EnginePressuresReport.msg"
  "msg/EngineReport.msg"
  "msg/FaultReport.msg"
  "msg/MiscReport.msg"
  "msg/Push2PassReport.msg"
  "msg/RaceControlHeader.msg"
  "msg/RdeTelemetry.msg"
  "msg/RoadLinePolynom.msg"
  "msg/RoadLinesPolynoms.msg"
  "msg/RvcTelemetry.msg"
  "msg/SteeringExtendedReport.msg"
  "msg/SteeringReport.msg"
  "msg/SlipAngleReport.msg"
  "msg/TargetTrajectoryCommand.msg"
  "msg/ThrottleReport.msg"
  "msg/TirePressureReport.msg"
  "msg/TireReport.msg"
  "msg/TireTempReport.msg"
  "msg/TireTempReports.msg"
  "msg/VehicleBitfields.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleControlCommand.msg"
  "msg/VehicleKinematicState.msg"
  "msg/VehicleManualControlCommand.msg"
  "msg/VehicleStatus.msg"
  "msg/VehicleTelemetry.msg"
  "msg/VehicleTiming.msg"
  "msg/WallBoundarySegment.msg"
  "msg/WheelPotentiometerReport.msg"
  "msg/WheelStrainGaugeReport.msg"
  "msg/RaceControlRestOfField.msg"
  "msg/WheelSpeedReport.msg"
  "msg/RacePathCommand.msg"
  "msg/RppTelemetry.msg"
  "msg/RppTrajectory.msg"
  "msg/RppPose.msg"
  "msg/LqgTelemetry.msg"
  "msg/TspTelemetry.msg"
  "msg/RideHeightReport.msg"
  "srv/UpdateVehicleModel.srv"
  "msg/PerceptionTelemetry.msg"
  "msg/PerceptionHealthTelemetry.msg"
  "msg/TrackedObjectTelemetry.msg"
  "msg/VehicleFlag.msg"

  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs gps_msgs sensor_msgs vision_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
