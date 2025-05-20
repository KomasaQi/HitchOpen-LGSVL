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


#ifndef RACE_VEHICLE_CONTROLLER__RVC_NODE_HPP_
#define RACE_VEHICLE_CONTROLLER__RVC_NODE_HPP_

#include <chrono>
#include <functional>
#include <string>
#include <fstream>
#include <thread>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_msgs/msg/vehicle_kinematic_state.hpp"
#include "race_msgs/msg/rvc_telemetry.hpp"
#include "race_msgs/msg/stop_type.hpp"
#include "race_msgs/msg/race_path_command.hpp"
#include "race_msgs/msg/wheel_speed_report.hpp"
#include "race_msgs/msg/engine_report.hpp"
#include "race_msgs/msg/push2_pass_report.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "vehicle_model/vehicle_model.hpp"
#include "external/pid_controller.hpp"
#include "race_vehicle_controller/rvc_state.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "base_common/cycle_profiler.hpp"

using race::ttl::Position;
using race::ttl::TtlIndex;
using race::ttl::SteeringLocation;
using race::ttl::Path;
using race::ttl::PathSharedPtr;

using race_msgs::msg::VehicleControlCommand;
using race_msgs::msg::TargetTrajectoryCommand;
using race_msgs::msg::VehicleManualControlCommand;
using race_msgs::msg::RvcTelemetry;
using race_msgs::msg::VehicleKinematicState;
using race_msgs::msg::StopType;
using race_msgs::msg::RacePathCommand;
using race_msgs::msg::WheelSpeedReport;
using race_msgs::msg::EngineReport;
using race_msgs::msg::Push2PassReport;

namespace race
{

class RvcNode : public rclcpp::Node
{
public:
  explicit RvcNode(const rclcpp::NodeOptions & options);

private:
  void step();

  void planner_timer_callback();

  void on_trajectory_update_received(const TargetTrajectoryCommand::SharedPtr msg);

  void on_vehicle_manual_command_received(const VehicleManualControlCommand::SharedPtr msg);

  void on_kinematic_state_received(const VehicleKinematicState::SharedPtr msg);

  void on_wheel_speed_received(const WheelSpeedReport::SharedPtr msg);

  void on_engine_report_received(const EngineReport::SharedPtr msg);

  void on_path(const RacePathCommand::SharedPtr msg);

  void on_push2pass_received(const Push2PassReport::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    std::vector<rclcpp::Parameter> const & parameters);

  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::TimerBase::SharedPtr planner_timer_;

  rclcpp::Time last_step_time_ {0, 0, RCL_CLOCK_UNINITIALIZED};

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // Subscribers (from RDE)

  rclcpp::Subscription<TargetTrajectoryCommand>::SharedPtr sub_trajectory_update_;
  rclcpp::Subscription<VehicleManualControlCommand>::SharedPtr sub_manual_control_;

  // Subscribers (from RPP)
  rclcpp::Subscription<RacePathCommand>::SharedPtr sub_path_;

  // Subscribers (from Sensors)

  rclcpp::Subscription<VehicleKinematicState>::SharedPtr sub_kin_state_;
  rclcpp::Subscription<WheelSpeedReport>::SharedPtr sub_wheel_speed_report_;
  rclcpp::Subscription<EngineReport>::SharedPtr sub_engine_report_;

  // Subscribers (from Push2Pass)
  rclcpp::Subscription<Push2PassReport>::SharedPtr sub_push2pass_report_;

  // Publishers (to Vehicke Interface)

  rclcpp::Publisher<VehicleControlCommand>::SharedPtr pub_raw_control_cmd_;

  // Publishers (to Telemetry)

  rclcpp::Publisher<RvcTelemetry>::SharedPtr pub_telemetry_;

  // Publishers (To Diagnostics)

  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diag_;

  // Class state
  RvcState::SharedPtr m_state_;
  RvcConfig::SharedPtr m_config_;
  race::vehicle_model::VehicleModel::SharedPtr m_model_;

  typedef std::vector<RvcPlugin::SharedPtr> PluginList;
  pluginlib::ClassLoader<RvcPlugin> m_plugin_loader_;
  PluginList m_plugins_;

  CycleProfiler<std::chrono::duration<double, std::milli>> m_profiler_;
};

}  // namespace race

#endif  // RACE_VEHICLE_CONTROLLER__RVC_NODE_HPP_
