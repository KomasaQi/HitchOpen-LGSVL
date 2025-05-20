// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__COMMON__INPUT_HPP_
#define RACE_DECISION_ENGINE__COMMON__INPUT_HPP_

#include <memory>

#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_msgs/msg/vehicle_command.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "race_msgs/msg/fault_report.hpp"
#include "race_msgs/msg/push2_pass_report.hpp"

#include "base_common/race_control.hpp"
#include "ttl.hpp"

namespace race::planning::race_decision_engine::common
{
struct CarState
{
  race::ttl::Position pos;
  double speed;
  double lap_distance;
  double lap_percentage;
  race::ttl::TtlIndex current_ttl_index;
  size_t current_ttl_wp_index;
  race::ttl::TtlIndex closest_ttl_index;
  size_t closest_ttl_wp_index;
};
struct RivalCar
{
  race::ttl::Position pos;
  double front_gap;
  double back_gap;
  double speed;
  race::ttl::TtlIndex current_ttl;
  bool exists;
};

struct RdeInputs
{
  typedef std::shared_ptr<RdeInputs> SharedPtr;
  typedef std::shared_ptr<const RdeInputs> ConstSharedPtr;
  typedef std::unique_ptr<RdeInputs> UniquePtr;

  race_msgs::msg::VehicleKinematicState::ConstSharedPtr localization_state_msg {};
  double localization_diff {0.0};
  race_msgs::msg::VehicleManualControlCommand::ConstSharedPtr input_manual_cmd_msg {};
  double input_manual_cmd_diff {0.0};
  race_msgs::msg::VehicleCommand::ConstSharedPtr race_control_msg {};
  double race_control_diff {0.0};
  autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr opp_car_detections_msg {};
  double opp_car_detections_diff {0.0};
  race_msgs::msg::FaultReport::ConstSharedPtr low_level_fault_report_msg {};
  double low_level_fault_report_diff {0.0};
  race_msgs::msg::Push2PassReport::ConstSharedPtr push2pass_report_msg {};
  double push2pass_report_diff {0.0};
};
}  // namespace race::planning::race_decision_engine::common

#endif  // RACE_DECISION_ENGINE__COMMON__INPUT_HPP_
