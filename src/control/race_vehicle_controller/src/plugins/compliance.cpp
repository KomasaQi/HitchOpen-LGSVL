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
 * @file compliance.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief Compliance with RDE
 * @version 0.2
 * @date 2022-05-15
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <memory>
#include <vector>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/target_trajectory_command.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "race_msgs/msg/stop_type.hpp"

using race_msgs::msg::StopType;
using race_msgs::msg::VehicleControlCommand;
using race_msgs::msg::TargetTrajectoryCommand;

namespace race
{
/**
 * @brief Compliance with RDE Commands and sanity check
 *
 */
class Compliance : public RvcPlugin
{
public:
  bool configure() override
  {
    m_max_throttle_cmd_ = node().declare_parameter<double>(
      "compliance.max_throttle_cmd", 100.0);

    m_max_brake_cmd_ = node().declare_parameter<double>(
      "compliance.max_brake_cmd", 100.0);

    m_min_brake_cmd_under_stop_ = node().declare_parameter<double>(
      "compliance.min_brake_cmd_under_stop", 10.0);
    return true;
  }

  bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    if (!const_state().all_input_received()) {
      return false;
    }
    switch (const_state().ttl_cmd->stop_type.stop_type) {
      case StopType::STOP_TYPE_IMMEDIATE:
      case StopType::STOP_TYPE_EMERGENCY:
        if (input_cmd.lon_control_type == VehicleControlCommand::LON_CONTROL_THROTTLE) {
          input_cmd.accelerator_cmd = 0.0;
          input_cmd.brake_cmd = m_min_brake_cmd_under_stop_;
        } else if (input_cmd.lon_control_type == VehicleControlCommand::LON_CONTROL_SPEED) {
          input_cmd.speed_cmd = 0.0;
        }
        break;
      case StopType::STOP_TYPE_NOMINAL:
        break;
    }

    // Check for nan and out-of-range in output
    auto check_invalid =
      [&](const char * what, double & val, const double & min, const double & max)
      {
        if (std::isnan(val)) {
          RCLCPP_ERROR_THROTTLE(
            node().get_logger(),
            *(node().get_clock()), 500, "%s output is nan! Overriding to zero.", what);
          val = 0.0;
        }
        if (val > max || val < min) {
          RCLCPP_ERROR_THROTTLE(
            node().get_logger(),
            *(node().get_clock()),
            500,
            "%s output of %lf is out of range [%lf, %lf]! Clipping.",
            what,
            val,
            min,
            max);
        }
        val = std::clamp(val, min, max);
      };

    if (input_cmd.lon_control_type == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      check_invalid("throttle", input_cmd.accelerator_cmd, 0.0, m_max_throttle_cmd_);
      check_invalid("brake", input_cmd.brake_cmd, 0.0, m_max_brake_cmd_);
    } else if (input_cmd.lon_control_type == VehicleControlCommand::LON_CONTROL_SPEED) {
      check_invalid("speed", input_cmd.speed_cmd, -100.0, 100.0);
    } else {
      RCLCPP_ERROR_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 500, "Unknown lon_control_type of %d", input_cmd.lon_control_type);
    }
    check_invalid(
      "steering", input_cmd.steering_cmd,
      -const_config().max_front_wheel_angle_rad, const_config().max_front_wheel_angle_rad);

    return true;
  }

  const char * get_plugin_name() override
  {
    return "Compliance Plugin";
  }

protected:
  double m_min_brake_cmd_under_stop_ = 10.0;
  double m_max_throttle_cmd_ = 100.0;
  double m_max_brake_cmd_ = 100.0;
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::Compliance, race::RvcPlugin)
