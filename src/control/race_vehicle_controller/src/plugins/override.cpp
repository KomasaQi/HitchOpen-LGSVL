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
 * @file override.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief Override with Joystick
 * @version 0.2
 * @date 2022-05-15
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_msgs/msg/vehicle_manual_control_command.hpp"
#include "race_vehicle_controller/plugin.hpp"

using race_msgs::msg::VehicleControlCommand;
using race_msgs::msg::VehicleManualControlCommand;

namespace race
{
struct OverrideConfig
{
  double throtte_threshold = 10.0;
  double brake_threshold = 5.0;
  double steer_threshold = 10.0;
  double max_manual_speed_mps = 2.5;
  double steer_decay_start = 10.0;
  double steer_decay_end = 15.0;
};

/**
 * @brief Manual Override
 *
 */
class Override : public RvcPlugin
{
public:
  bool configure() override
  {
    m_config_ = OverrideConfig{
      node().declare_parameter<double>("override.throtte_threshold"),
      node().declare_parameter<double>("override.brake_threshold"),
      node().declare_parameter<double>("override.steer_threshold_deg") * M_PI / 180.0,
      node().declare_parameter<double>("override.max_manual_speed_mps"),
      node().declare_parameter<double>("override.steer_decay.start_speed_mps"),
      node().declare_parameter<double>("override.steer_decay.end_speed_mps"),
    };
    return true;
  }

  bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    if (!const_state().all_input_received()) {
      return false;
    }
    // Full autonomous. No override.
    // Update 5/15/2022 this is stupid. Commented out.
    // if (!allow_override_) {
    //   return;
    // }

    // Manual mode. full override.
    const auto & man_cmd = *const_state().input_manual_cmd;
    if (man_cmd.use_manual_cmd) {
      input_cmd.accelerator_cmd = man_cmd.vehicle_control_command.accelerator_cmd;
      input_cmd.brake_cmd = man_cmd.vehicle_control_command.brake_cmd;
      input_cmd.steering_cmd = man_cmd.vehicle_control_command.steering_cmd;
      input_cmd.speed_cmd = man_cmd.vehicle_control_command.speed_cmd;
      input_cmd.push2pass_cmd = man_cmd.vehicle_control_command.push2pass_cmd;
      if (abs(input_cmd.speed_cmd) > m_config_.max_manual_speed_mps) {
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *(node().get_clock()), 500, "Clipping speed command to max manual speed.");
        input_cmd.speed_cmd = std::clamp(
          input_cmd.speed_cmd, -m_config_.max_manual_speed_mps,
          m_config_.max_manual_speed_mps);
      }

      if (input_cmd.brake_cmd > 0.0) {
        input_cmd.accelerator_cmd = 0.0;
      } else if (const_state().kin_state->speed_mps > m_config_.max_manual_speed_mps) {
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *(node().get_clock()), 500, "Max manual speed reached. Overriding throttle to zero.");
        input_cmd.accelerator_cmd = 0.0;
      } else if (input_cmd.accelerator_cmd > 0.0) {
        input_cmd.brake_cmd = 0.0;
      }
      return true;
    }

    // Autonomous override below

    if (input_cmd.lon_control_type == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      // Throttle control mode
      const auto & man_acc = man_cmd.vehicle_control_command.accelerator_cmd;
      const auto & man_brk = man_cmd.vehicle_control_command.brake_cmd;
      // In limit-auto-throttle mode: autonomous throttle scaled by manual throttle level.
      if (man_cmd.limit_auto_throttle) {
        static constexpr auto MAX_THROTTLE_CMD = 100.0;
        input_cmd.accelerator_cmd =
          input_cmd.accelerator_cmd * std::clamp(
          man_acc / MAX_THROTTLE_CMD, 0.0, 1.0);
        if (input_cmd.accelerator_cmd > 0.0) {
          input_cmd.brake_cmd = 0.0;
        }
      } else if (man_acc > m_config_.throtte_threshold) {
        // In auto-throttle mode: autonomous throttle passthrough unless override.
        // first check if the autonomous input intends to accelerate (brake = 0.0)
        if (input_cmd.brake_cmd <= 0.0) {
          input_cmd.accelerator_cmd = std::min(input_cmd.accelerator_cmd, man_acc);
          input_cmd.brake_cmd = 0.0;
        }
      }
      // Autonomous brake override
      if (man_brk > m_config_.brake_threshold) {
        input_cmd.brake_cmd = std::max(input_cmd.brake_cmd, man_brk);
        input_cmd.accelerator_cmd = 0.0;
      }
    } else if (input_cmd.lon_control_type == VehicleControlCommand::LON_CONTROL_SPEED) {
      // Speed control mode
      // In limit-auto-speed mode: autonomous speed scaled by manual speed level.
      if (man_cmd.limit_auto_throttle) {
        input_cmd.speed_cmd =
          input_cmd.speed_cmd * std::clamp(
          man_cmd.vehicle_control_command.speed_cmd / m_config_.max_manual_speed_mps, 0.0, 1.0);
      } else if (man_cmd.vehicle_control_command.speed_cmd > m_config_.throtte_threshold) {
        // In auto-throttle mode: autonomous throttle passthrough unless override.
        input_cmd.speed_cmd = man_cmd.vehicle_control_command.speed_cmd;
      }
    }
    // Autonomous steer override
    const auto & man_steer = man_cmd.vehicle_control_command.steering_cmd;
    if (abs(man_steer) > m_config_.steer_threshold) {
      input_cmd.steering_cmd = calc_steer_override(
        const_state().kin_state->speed_mps, man_steer, input_cmd.steering_cmd);
    }
    return true;
  }

  const char * get_plugin_name() override
  {
    return "Override Plugin";
  }

private:
  OverrideConfig m_config_;

  double calc_steer_override(
    const double & speed_mps, const double & man_steer,
    const double & auto_steer)
  {
    if (speed_mps < m_config_.steer_decay_start) {
      return man_steer;
    }

    if (speed_mps > m_config_.steer_decay_end) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 500, "Steer cannot be overrided. Speed too high.");
      return auto_steer;
    }

    // the decay follows -0.5 sin(x) + 0.5 in [-pi/2, pi/2]
    // such that the scale goes from 1 to 0.
    const auto x = M_PI / (m_config_.steer_decay_end - m_config_.steer_decay_start) *
      (speed_mps - m_config_.steer_decay_start) - M_PI_2;
    return man_steer * (-0.5 * sin(x) + 0.5);
  }
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::Override, race::RvcPlugin)
