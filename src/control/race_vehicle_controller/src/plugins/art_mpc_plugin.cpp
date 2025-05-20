// Copyright 2023 AI Racing Tech
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
// Defines the MPC plugin to be used in the RVC

#include <limits>
#include <algorithm>

#include "lmpc_msgs/msg/mpc_telemetry.hpp"

#include "race_vehicle_controller/plugin.hpp"

namespace race
{
using lmpc_msgs::msg::MPCTelemetry;
using std::placeholders::_1;

class ArtMpcPlugin : public RvcPlugin
{
public:
  bool configure() override
  {
    mpc_telemetry_sub_ = node().create_subscription<MPCTelemetry>(
      "mpc_telemetry", 1, std::bind(&ArtMpcPlugin::mpc_telemetry_callback, this, _1));
    mpc_cmd_sub_ = node().create_subscription<VehicleControlCommand>(
      "mpc_cmd", 1, std::bind(&ArtMpcPlugin::mpc_cmd_callback, this, _1));
    max_mpc_invalid_count_ = node().declare_parameter<int>("art_mpc.max_mpc_invalid_count");
    mpc_invalid_count_threshold_ = node().declare_parameter<int>(
      "art_mpc.mpc_invalid_count_threshold");
    mpc_engage_speed = node().declare_parameter<double>("art_mpc.mpc_engage_speed");
    mpc_disengage_speed = node().declare_parameter<double>("art_mpc.mpc_disengage_speed");
    return true;
  }

  void on_kinematic_update() override
  {
  }

  void on_ttl_command_update() override
  {
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (!const_state().all_input_received()) {
      return true;
    }

    if (mpc_telemetry_ && mpc_telemetry_->solved) {
      mpc_invalid_count_ = std::max(0, mpc_invalid_count_ - 1);
    } else {
      mpc_invalid_count_ = std::min(max_mpc_invalid_count_, mpc_invalid_count_ + 1);
    }

    if (mpc_telemetry_ &&
      (node().now() - rclcpp::Time(mpc_telemetry_->header.stamp)).seconds() > 0.2)
    {
      mpc_invalid_count_ = max_mpc_invalid_count_;
    }

    output_cmd.steering_cmd = std::numeric_limits<double>::quiet_NaN();
    output_cmd.accelerator_cmd = std::numeric_limits<double>::quiet_NaN();
    output_cmd.brake_cmd = std::numeric_limits<double>::quiet_NaN();
    state().telemetry->mpc_enabled = false;

    if (const_state().kin_state->speed_mps < mpc_disengage_speed &&
      const_state().ttl_cmd->target_speed < mpc_engage_speed)
    {
      // if engage speed < speed < disengage speed,
      // and target speed is smaller than engage speed, use backup controller
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "Target speed is less than mpc engage speed. Backup controller should override.");
    } else if (mpc_telemetry_ && mpc_telemetry_->solve_time > 100) {
      mpc_invalid_count_ = max_mpc_invalid_count_;
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "MPC solution took too long. Backup controller should override.");
    } else if (state().ttl_cmd->in_pit_ttl) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "In pit TTL. Backup controller should override.");
    } else if (mpc_invalid_count_ > mpc_invalid_count_threshold_ || !mpc_cmd_) {
      mpc_invalid_count_ = max_mpc_invalid_count_;
      // if mpc command is not valid, just use the backup controller
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "MPC solution is invalid. Backup controller should override.");
    } else if (const_state().kin_state->speed_mps < mpc_engage_speed) {
      // if speed is less than mpc engage speed, use backup controller
      RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "Current speed is less than mpc engage speed. Backup controller should override.");
    } else {
      output_cmd.steering_cmd = mpc_cmd_->steering_cmd;
      output_cmd.accelerator_cmd = mpc_cmd_->accelerator_cmd;
      output_cmd.brake_cmd = mpc_cmd_->brake_cmd;
      state().telemetry->steering_cmd = mpc_cmd_->steering_cmd;
      state().telemetry->throttle_cmd = mpc_cmd_->accelerator_cmd;
      state().telemetry->brake_cmd = mpc_cmd_->brake_cmd;
      // Lateral error is intentionally commented out
      // state().telemetry->lateral_error = mpc_telemetry_->state[1];
      state().telemetry->mpc_enabled = true;
    }

    return true;
  }

  const char * get_plugin_name() override
  {
    return "ART MPC Plugin";
  }

private:
  rclcpp::Subscription<MPCTelemetry>::SharedPtr mpc_telemetry_sub_;
  rclcpp::Subscription<VehicleControlCommand>::SharedPtr mpc_cmd_sub_;
  VehicleControlCommand::SharedPtr mpc_cmd_;
  MPCTelemetry::SharedPtr mpc_telemetry_;
  int mpc_invalid_count_ = 0;
  int max_mpc_invalid_count_ = 20;
  int mpc_invalid_count_threshold_ = 10;
  double mpc_engage_speed = 10.0;
  double mpc_disengage_speed = 15.0;

  void mpc_telemetry_callback(const MPCTelemetry::SharedPtr msg)
  {
    mpc_telemetry_ = msg;
  }

  void mpc_cmd_callback(const VehicleControlCommand::SharedPtr msg)
  {
    mpc_cmd_ = msg;
  }
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::ArtMpcPlugin, race::RvcPlugin)
