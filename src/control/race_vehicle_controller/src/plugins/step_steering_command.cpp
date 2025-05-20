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
//
// Creates paths for the RVC to track

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

#include "race_vehicle_controller/plugin.hpp"

using race::ttl::Position;

namespace race
{
class StepSteeringCommand : public RvcPlugin
{
public:
  bool configure() override
  {
    // Declare parameters
    max_steer_ = node().declare_parameter<double>("step_steering_command.max_steer", 1.0);
    step_size_ = node().declare_parameter<double>("step_steering_command.step_size", 1.0);
    step_width_ = node().declare_parameter<double>("step_steering_command.step_width", 1.0);
    is_active_ = node().declare_parameter<bool>("step_steering_command.activated", false);
    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    const auto & name = param.get_name();
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "step_steering_command.max_steer") {
        max_steer_ = param.as_double();
        result.successful = true;
      } else if (name == "step_steering_command.step_size") {
        step_size_ = param.as_double();
        result.successful = true;
      } else if (name == "step_steering_command.step_width") {
        step_width_ = param.as_double();
        result.successful = true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "step_steering_command.activated") {
        bool new_is_active = param.as_bool();
        if (new_is_active && !previous_active_state_) {
          initialized_ = false;
        }
        is_active_ = new_is_active;
        previous_active_state_ = new_is_active;
        result.successful = true;
      }
    }
    return result.successful;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    if (!is_active_) {
      return true;
    }
    if (!initialized_) {
      start_time_ = node().now();
      initialized_ = true;
      current_step_ = 0;
    }

    rclcpp::Time now = node().now();
    double time_seconds = (now - start_time_).seconds();
    int total_steps = step_sequence_.size();
    int step_index = static_cast<int>(time_seconds / step_width_) % total_steps;
    if (step_index != current_step_) {
      current_step_ = step_index;
    }

    double steering_angle = step_sequence_[current_step_] * max_steer_;

    output_cmd.steering_cmd = output_cmd.steering_cmd + steering_angle;

    return true;
  }

  const char * get_plugin_name() override
  {
    return "StepSteeringCommand";
  }

private:
  double max_steer_;  // Maximum steering angle
  double step_size_;  // Step size for the steering angle
  double step_width_;  // Time duration for each step
  bool is_active_;

  rclcpp::Time start_time_;
  bool initialized_ = false;
  bool previous_active_state_ = false;

  int current_step_ = 0;
  std::vector<double> step_sequence_ = {1.0, 0.0, -1.0, 0.0};  // Step sequence (normalized)
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::StepSteeringCommand, race::RvcPlugin)
