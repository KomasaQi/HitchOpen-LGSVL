// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__COMMON__OUTPUT_HPP_
#define RACE_DECISION_ENGINE__COMMON__OUTPUT_HPP_

#include <memory>

#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_msgs/msg/rde_telemetry.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace common
{
struct RdeOutputs
{
  typedef std::shared_ptr<RdeOutputs> SharedPtr;
  typedef std::shared_ptr<const RdeOutputs> ConstSharedPtr;
  typedef std::unique_ptr<RdeOutputs> UniquePtr;

  race_msgs::msg::VehicleManualControlCommand::SharedPtr output_manual_cmd_msg {};
  race_msgs::msg::TargetTrajectoryCommand::SharedPtr ttc_msg {};
  race_msgs::msg::RdeTelemetry::SharedPtr rde_telem_msg {};
};
}  // namespace common
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__COMMON__OUTPUT_HPP_
