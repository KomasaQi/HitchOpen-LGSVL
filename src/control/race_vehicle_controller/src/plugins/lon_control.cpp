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
// Author: Haoru Xue (haorux@andrew.cmu.edu)
// Defines a RVC plugin for control throttle and brake

#include <math.h>

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ttl.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "race_vehicle_controller/external/pid_controller.hpp"
#include "race_vehicle_controller/rvc_utils.hpp"

namespace race
{
using ttl::Position;
struct LonConfig
{
  PidCoefficients throttle_pid_param;
  PidCoefficients brake_pid_param;
  double brake_deadband_to_speed_ratio;
  double throttle_max_to_speed_ratio;
  double parking_brake_command;
  uint8_t lon_control_type = 0;
  std::vector<double> ss_lookup_speed;
  std::vector<double> ss_lookup_throttle;
};
struct PidLonControlOutput
{
  double throttle = 0.0;
  double brake = 0.0;
};
struct LonControlState
{
  double target_speed = 0.0;
  double current_speed = 0.0;
  double throttle_error = 0.0;
  double brake_error = 0.0;
  double throttle_pid_output = 0.0;
  double brake_pid_output = 0.0;
  PidController throttle_pid;
  PidController brake_pid;
  PidLonControlOutput output;
  rclcpp::Time last_pid_time;
  rclcpp::TimerBase::SharedPtr pid_timer;
};

class LonControl : public RvcPlugin
{
public:
  typedef std::shared_ptr<LonControl> SharedPtr;
  bool configure() override
  {
    config_ = LonConfig{
      PidCoefficients{
        node().declare_parameter<double>("lon_control.throttle_pid.kp", 1.0),
        node().declare_parameter<double>("lon_control.throttle_pid.ki", 0.1),
        node().declare_parameter<double>("lon_control.throttle_pid.kd", 0.1),
        node().declare_parameter<double>("lon_control.throttle_pid.min_cmd", 0.0),
        node().declare_parameter<double>("lon_control.throttle_pid.max_cmd", 50.0),
        node().declare_parameter<double>("lon_control.throttle_pid.min_i", -10.0),
        node().declare_parameter<double>("lon_control.throttle_pid.max_i", 10.0),
      },
      PidCoefficients{
        node().declare_parameter<double>("lon_control.brake_pid.kp", 1.0),
        node().declare_parameter<double>("lon_control.brake_pid.ki", 0.1),
        node().declare_parameter<double>("lon_control.brake_pid.kd", 0.1),
        node().declare_parameter<double>("lon_control.brake_pid.min_cmd", 0.0),
        node().declare_parameter<double>("lon_control.brake_pid.max_cmd", 50.0),
        node().declare_parameter<double>("lon_control.brake_pid.min_i", -10.0),
        node().declare_parameter<double>("lon_control.brake_pid.max_i", 10.0),
      },
      node().declare_parameter<double>("lon_control.brake_deadband_to_speed_ratio", 0.2),
      node().declare_parameter<double>("lon_control.throttle_max_to_speed_ratio", 2.5),
      node().declare_parameter<double>("lon_control.parking_brake_command", 50.0),
      static_cast<uint8_t>(
        node().declare_parameter<int64_t>("lon_control.lon_control_type", 0)),
      node().declare_parameter(
        "lon_control.steady_state_throttle_lookup.speed",
        std::vector<double>{}),
      node().declare_parameter(
        "lon_control.steady_state_throttle_lookup.throttle",
        std::vector<double>{})
    };

    as_backup_ = node().declare_parameter("lon_control.as_backup", false),


    lon_state().throttle_pid = PidController("throttle.pid", config_.throttle_pid_param);
    lon_state().brake_pid = PidController("brake.pid", config_.brake_pid_param);
    lon_state().last_pid_time = node().now();
    lon_state().pid_timer = rclcpp::create_timer(
      &node(), node().get_clock(), std::chrono::duration<float>(
        node().declare_parameter<float>("lon_control.pid_interval_sec", 0.01)), [this] {update();});

    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    if (name == "lon_control.throttle_pid.kp") {
      return lon_state().throttle_pid.try_update_param("kp", param);
    } else if (name == "lon_control.throttle_pid.ki") {
      return lon_state().throttle_pid.try_update_param("ki", param);
    } else if (name == "lon_control.throttle_pid.kd") {
      return lon_state().throttle_pid.try_update_param("kd", param);
    } else if (name == "lon_control.throttle_pid.min_cmd") {
      return lon_state().throttle_pid.try_update_param("min_cmd", param);
    } else if (name == "lon_control.throttle_pid.max_cmd") {
      return lon_state().throttle_pid.try_update_param("max_cmd", param);
    } else if (name == "lon_control.throttle_pid.min_i") {
      return lon_state().throttle_pid.try_update_param("min_i", param);
    } else if (name == "lon_control.throttle_pid.max_i") {
      return lon_state().throttle_pid.try_update_param("max_i", param);
    } else if (name == "lon_control.brake_pid.kp") {
      return lon_state().throttle_pid.try_update_param("kp", param);
    } else if (name == "lon_control.brake_pid.ki") {
      return lon_state().throttle_pid.try_update_param("ki", param);
    } else if (name == "lon_control.brake_pid.kd") {
      return lon_state().throttle_pid.try_update_param("kd", param);
    } else if (name == "lon_control.brake_pid.min_cmd") {
      return lon_state().throttle_pid.try_update_param("min_cmd", param);
    } else if (name == "lon_control.brake_pid.max_cmd") {
      return lon_state().throttle_pid.try_update_param("max_cmd", param);
    } else if (name == "lon_control.brake_pid.min_i") {
      return lon_state().throttle_pid.try_update_param("min_i", param);
    } else if (name == "lon_control.brake_pid.max_i") {
      return lon_state().throttle_pid.try_update_param("max_i", param);
    } else if (name == "lon_control.brake_deadband_to_speed_ratio") {
      lon_config().brake_deadband_to_speed_ratio = param.as_double();
    } else if (name == "lon_control.throttle_max_to_speed_ratio") {
      lon_config().throttle_max_to_speed_ratio = param.as_double();
    } else if (name == "lon_control.parking_brake_command") {
      lon_config().parking_brake_command = param.as_double();
    } else {
      return false;
    }
    return true;
  }

  void on_kinematic_update() override
  {
    lon_state().current_speed = const_state().kin_state->speed_mps;
  }

  void on_trajectory_update() override
  {
    if (const_state().path->size() > 0) {
      lon_state().target_speed = const_state().path->back().target_speed;
    }
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (as_backup_) {
      return true;
    }
    if (config_.lon_control_type == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      output_cmd.accelerator_cmd = lon_state().output.throttle;
      output_cmd.brake_cmd = lon_state().output.brake;
    } else {
      output_cmd.speed_cmd = lon_state().target_speed;
      output_cmd.lon_control_type = VehicleControlCommand::LON_CONTROL_SPEED;
    }
    return true;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    // Checks if primary controller failed and this is a backup controller
    if (as_backup_ && isnan(output_cmd.steering_cmd)) {
      if (config_.lon_control_type == VehicleControlCommand::LON_CONTROL_THROTTLE) {
        output_cmd.accelerator_cmd = lon_state().output.throttle;
        output_cmd.brake_cmd = lon_state().output.brake;
      } else {
        output_cmd.speed_cmd = lon_state().target_speed;
        output_cmd.lon_control_type = VehicleControlCommand::LON_CONTROL_SPEED;
      }
    }
    return true;
  }


  void reset_pid_integrals()
  {
    state_.throttle_pid.reset_integral_error(0.0);
    state_.brake_pid.reset_integral_error(0.0);
  }

  LonControlState & lon_state()
  {
    return state_;
  }

  LonConfig & lon_config()
  {
    return config_;
  }

  const char * get_plugin_name() override
  {
    return "Lon Control Plugin";
  }

  void update()
  {
    // TODO(haoru): this can be definitely cleaned up
    static auto this_pid_time = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
    if (this_pid_time.get_clock_type() == RCL_CLOCK_UNINITIALIZED) {
      this_pid_time = node().now();
      return;
    }
    this_pid_time = node().now();
    if (!const_state().all_input_received() || !const_state().path) {
      lon_state().last_pid_time = this_pid_time;
      return;
    }

    const auto dt = (this_pid_time - lon_state().last_pid_time).seconds();
    const auto speed_error = lon_state().target_speed - lon_state().current_speed;
    auto throttle_error = speed_error;
    auto brake_error = -speed_error;
    static constexpr auto RESET_INTEGRAL_BELOW = 1.0;
    if (lon_state().current_speed < RESET_INTEGRAL_BELOW) {
      reset_pid_integrals();
    }
    double throttle_cmd = state_.throttle_pid.update(throttle_error, dt);
    throttle_cmd += utils::interpolate(
      lon_config().ss_lookup_speed,
      lon_config().ss_lookup_throttle, lon_state().current_speed, false);
    throttle_cmd = std::clamp(throttle_cmd, 0.0, 100.0);
    double brake_cmd = state_.brake_pid.update(brake_error, dt);
    lon_state().brake_pid_output = brake_cmd;

    // Traction control
    static constexpr auto LOW_SPEED_THROTTLE_MIN = 30.0;
    const auto throttle_cap = std::max(
      LOW_SPEED_THROTTLE_MIN,
      lon_config().throttle_max_to_speed_ratio * lon_state().target_speed);
    throttle_cmd = std::min(throttle_cmd, throttle_cap);

    // Checkout brake deadband to determine either brake or speedup
    static constexpr auto MIN_BRAKE_DEADBAND_MPS = 0.5;
    if (brake_error >
      std::max(
        config_.brake_deadband_to_speed_ratio * lon_state().current_speed,
        MIN_BRAKE_DEADBAND_MPS))
    {
      lon_state().output.brake = brake_cmd;
      lon_state().output.throttle = 0.0;
    } else {
      lon_state().output.throttle = throttle_cmd;
      lon_state().output.brake = 0.0;
    }
    // Edge condition: when speed is smaller than brake deadband
    if (lon_state().current_speed <= MIN_BRAKE_DEADBAND_MPS) {
      static constexpr auto MIN_BRAKE_VALUE = 1.0;
      // Brake when above minimum brake threshold
      if (brake_cmd > MIN_BRAKE_VALUE) {
        lon_state().output.brake = brake_cmd;
        lon_state().output.throttle = 0.0;
      } else {
        lon_state().output.throttle = throttle_cmd;
        lon_state().output.brake = 0.0;
      }
    }
    // Edge condition: when stopping, hold parking brake
    if (lon_state().current_speed < 1.0 && lon_state().target_speed < 1.0) {
      RCLCPP_INFO_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 1000, "Vehicle stopped and target speed zero. Holding brake.");
      lon_state().output.throttle = 0.0;
      lon_state().output.brake = lon_config().parking_brake_command;
    }
    lon_state().last_pid_time = this_pid_time;
  }

private:
  LonConfig config_;
  LonControlState state_;
  bool as_backup_ {false};
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::LonControl, race::RvcPlugin)
