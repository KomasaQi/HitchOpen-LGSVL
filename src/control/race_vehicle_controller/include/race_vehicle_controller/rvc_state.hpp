// Copyright 2022 AI Racing Tech
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


/**
 * @file rvc_state.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief rvc state struct
 * @version 0.1
 * @date 2022-05-14
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef RACE_VEHICLE_CONTROLLER__RVC_STATE_HPP_
#define RACE_VEHICLE_CONTROLLER__RVC_STATE_HPP_

#include <memory>
#include <string>

#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/engine_report.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/rvc_telemetry.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/push2_pass_report.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "ttl.hpp"

namespace race
{
using race_msgs::msg::VehicleControlCommand;
using race_msgs::msg::EngineReport;
using race_msgs::msg::VehicleManualControlCommand;
using race_msgs::msg::TargetTrajectoryCommand;
using race_msgs::msg::WheelSpeedReport;
using race_msgs::msg::VehicleKinematicState;
using race_msgs::msg::RvcTelemetry;
using race_msgs::msg::WheelSpeedReport;
using race_msgs::msg::Push2PassReport;

using diagnostic_msgs::msg::DiagnosticArray;

using race::ttl::Path;
using race::ttl::PathSharedPtr;

struct RvcState
{
  typedef std::shared_ptr<RvcState> SharedPtr;
  typedef std::unique_ptr<RvcState> UniquePtr;
  VehicleKinematicState::SharedPtr kin_state {};
  WheelSpeedReport::SharedPtr wheel_speed_report {};
  EngineReport::SharedPtr engine_report {};
  VehicleControlCommand::SharedPtr output_cmd {};
  VehicleManualControlCommand::SharedPtr input_manual_cmd {};
  TargetTrajectoryCommand::SharedPtr ttl_cmd {};
  RvcTelemetry::SharedPtr telemetry {};
  PathSharedPtr path {};
  DiagnosticArray::SharedPtr diagnostics {};
  Push2PassReport::SharedPtr push2pass_report {};

  bool all_input_received() const
  {
    return kin_state && input_manual_cmd && ttl_cmd;
  }
};

struct RvcConfig
{
  typedef std::shared_ptr<RvcConfig> SharedPtr;
  typedef std::unique_ptr<RvcConfig> UniquePtr;
  double control_output_interval_sec;
  double max_front_wheel_angle_rad;
  double wheelbase_m;   // Front-rear axle distance
  double track_m;   // Wheel-wheel distance on the same axle
  double vehicle_weight_kg;   // Vehicle weight
  double turn_left_bias_rad;  // left turn bias
  std::string ttl_directory;
};
}  // namespace race

#endif  // RACE_VEHICLE_CONTROLLER__RVC_STATE_HPP_
