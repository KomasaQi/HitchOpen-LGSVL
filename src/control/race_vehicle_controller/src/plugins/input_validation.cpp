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
 * @file input_validation.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief input validation
 * @version 0.2
 * @date 2022-05-14
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include <sstream>

#include "race_vehicle_controller/plugin.hpp"

#include "race_msgs/msg/stop_type.hpp"
using race_msgs::msg::StopType;

namespace race
{
class InputValidation : public RvcPlugin
{
public:
  bool configure() override
  {
    return true;
  }
  void on_kinematic_update() override
  {
    const auto & pose = const_state().kin_state->pose.pose;
    bool pose_valid = !isnan(pose.position.x) &&
      !isnan(pose.position.y) &&
      !isnan(pose.position.z) &&
      !isnan(pose.orientation.x) &&
      !isnan(pose.orientation.y) &&
      !isnan(pose.orientation.z) &&
      !isnan(pose.orientation.w);
    bool speed_valid = !isnan(const_state().kin_state->speed_mps);
    bool acc_valid = !isnan(const_state().kin_state->accel.accel.linear.x) &&
      !isnan(const_state().kin_state->accel.accel.linear.y) &&
      !isnan(const_state().kin_state->accel.accel.linear.z);
    kin_valid = pose_valid && speed_valid && acc_valid;
  }
  void on_ttl_command_update() override
  {
    ttl_valid = !isnan(const_state().ttl_cmd->target_speed) &&
      !isnan(const_state().ttl_cmd->target_speed_rate) &&
      !isnan(const_state().ttl_cmd->stop_type.stop_type);
  }
  void on_manual_command_update() override
  {
    const auto & cmd = const_state().input_manual_cmd->vehicle_control_command;
    manual_valid = !isnan(cmd.accelerator_cmd) &&
      !isnan(cmd.brake_cmd) &&
      !isnan(cmd.gear_cmd) &&
      !isnan(cmd.speed_cmd) &&
      !isnan(cmd.steering_cmd);
  }
  void on_trajectory_update() override
  {
    traj_valid = const_state().path && const_state().path->size() > 0;
  }
  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    (void) output_cmd;
    const bool all_input_valid = kin_valid && ttl_valid && manual_valid && traj_valid;
    if (!all_input_valid) {
      std::stringstream ss;
      ss << "Some input is not valid! Flags: kin_valid [" << kin_valid << "] ttl_valid [" <<
        ttl_valid << "] manual_valid [" << manual_valid << "] traj_valid [" << traj_valid << "]";
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 500, ss.str().c_str());

      if (manual_valid && const_state().input_manual_cmd->use_manual_cmd) {
        // special case: some sensor is not valid, but teleop is still permitted
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *node().get_clock(), 500, "Manual command valid. Teleop permitted.");
        return true;
      }
    }
    return all_input_valid;
  }

  const char * get_plugin_name() override
  {
    return "Input Validation Plugin";
  }

private:
  bool kin_valid = false;
  bool ttl_valid = false;
  bool manual_valid = false;
  bool traj_valid = false;
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::InputValidation, race::RvcPlugin)
