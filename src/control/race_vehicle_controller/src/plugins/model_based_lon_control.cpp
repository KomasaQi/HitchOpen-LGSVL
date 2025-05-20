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

#include <memory>

#include "vehicle_model/vehicle_model.hpp"
#include "vehicle_model/ros_param_loader.hpp"
#include "race_vehicle_controller/external/pid_controller.hpp"
#include "base_common/low_pass_filter.hpp"
#include "race_vehicle_controller/plugin.hpp"

namespace race
{
class ModelBasedLonControl : public RvcPlugin
{
public:
  bool configure() override
  {
    as_backup_ = node().declare_parameter<bool>("model_lon_control.as_backup");
    acc_pid_ = PidController(
      "acc_pid", PidCoefficients{
        node().declare_parameter<double>("model_lon_control.acc_pid.kp", 1.0),
        node().declare_parameter<double>("model_lon_control.acc_pid.ki", 0.1),
        node().declare_parameter<double>("model_lon_control.acc_pid.kd", 0.1),
        node().declare_parameter<double>("model_lon_control.acc_pid.min_cmd", -50.0),
        node().declare_parameter<double>("model_lon_control.acc_pid.max_cmd", 50.0),
        node().declare_parameter<double>("model_lon_control.acc_pid.min_i", -10.0),
        node().declare_parameter<double>("model_lon_control.acc_pid.max_i", 10.0)});
    brake_deadband_psi_ =
      node().declare_parameter<double>("model_lon_control.brake_deadband_psi", 100.0);
    parking_brake_psi_ =
      node().declare_parameter<double>("model_lon_control.parking_brake_psi", 1000.0);
    stopping_speed_ =
      node().declare_parameter<double>("model_lon_control.stopping_speed", 2.0);
    lon_control_type_ =
      static_cast<uint8_t>(node().declare_parameter<int64_t>(
        "model_lon_control.lon_control_type",
        0));
    throttle_scale_ =
      node().declare_parameter<double>("model_lon_control.throttle_scale", 1.0);
    throttle_max_ =
      node().declare_parameter<double>("model_lon_control.throttle_max", 100.0);
    norminal_max_dcc_ =
      node().declare_parameter<double>("model_lon_control.norminal_max_dcc", -5.0);
    follow_mode_max_speed_diff_ = node().declare_parameter<double>(
      "follow_mode.max_speed_diff_mps",
      5.0);
    follow_mode_max_gap_diff_ratio_ = node().declare_parameter<double>(
      "follow_mode.gap_diff_ratio",
      0.2);
    test_mode_ = node().declare_parameter<bool>("model_lon_control.test_mode.enabled", false);
    max_brake_ = node().declare_parameter<double>(
      "model_lon_control.test_mode.max_brake");
    const_max_brake_ = node().declare_parameter<bool>(
      "model_lon_control.test_mode.const_max_brake", false);

    const auto output_cutoff_freq = node().declare_parameter<double>(
      "model_lon_control.output_cutoff_freq", 3.0);
    throttle_filter_ = LowPassFilter(
      output_cutoff_freq,
      const_config().control_output_interval_sec
    );
    brake_filter_ = LowPassFilter(
      output_cutoff_freq,
      const_config().control_output_interval_sec
    );
    return true;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (!const_state().all_input_received() || !const_state().path) {
      return true;
    }

    if (as_backup_) {
      return true;
    }

    calc_lon_control(output_cmd);
    return true;
  }

  bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    if (!const_state().all_input_received() || !const_state().path) {
      return true;
    }

    if (!as_backup_) {
      return true;
    }

    if (lon_control_type_ == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      if (isnan(input_cmd.accelerator_cmd) || isnan(input_cmd.brake_cmd)) {
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *(node().get_clock()), 3000,
          "NaN throttle and brake detected. Model based lon control overriding.");
        calc_lon_control(input_cmd);
      }
    } else if (lon_control_type_ == VehicleControlCommand::LON_CONTROL_SPEED) {
      if (isnan(input_cmd.speed_cmd)) {
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *(node().get_clock()), 3000, "NaN speed detected. Model based lon control overriding.");
        calc_lon_control(input_cmd);
      }
    }
    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    if (name == "model_lon_control.throttle_scale") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() > 0.0) {
          throttle_scale_ = param.as_double();
          return true;
        }
      }
    } else if (name == "model_lon_control.throttle_max") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0) {
          throttle_max_ = param.as_double();
          return true;
        }
      }
    } else if (name == "model_lon_control.norminal_max_dcc") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() <= 0.0) {
          norminal_max_dcc_ = param.as_double();
          return true;
        }
      }
    } else if (name == "model_lon_control.test_mode.enabled") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        test_mode_ = param.as_bool();
        return true;
      }
    } else if (name == "model_lon_control.test_mode.max_brake") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        max_brake_ = param.as_double();
        return true;
      }
    } else if (name == "model_lon_control.test_mode.const_max_brake") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        const_max_brake_ = param.as_bool();
        return true;
      }
    }
    return false;
  }

private:
  PidController acc_pid_;
  double brake_deadband_psi_;
  double parking_brake_psi_;
  double stopping_speed_;
  double throttle_scale_;
  double throttle_max_;
  double norminal_max_dcc_;
  double follow_mode_max_speed_diff_;
  double follow_mode_max_gap_diff_ratio_;
  double slip_angle_ = 0.0;
  double max_brake_ = 0.0;
  LowPassFilter throttle_filter_;
  LowPassFilter brake_filter_;
  uint8_t lon_control_type_;
  bool as_backup_ = false;
  bool test_mode_ = false;
  bool const_max_brake_ = false;

  void calc_lon_control(VehicleControlCommand & output_cmd)
  {
    // set goals
    const auto & stop_type = const_state().ttl_cmd->stop_type.stop_type;
    const auto & current_speed = const_state().kin_state->speed_mps;
    auto target_speed = const_state().path->back().target_speed;
    if (stop_type != race_msgs::msg::StopType::STOP_TYPE_NOMINAL) {
      target_speed = 0.0;
      output_cmd.push2pass_cmd = false;
    }

    // calculate control
    if (lon_control_type_ == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      if (const_state().engine_report) {
        // Update goal
        if (current_speed < stopping_speed_) {
          acc_pid_.reset_integral_error(0.0);
        }
        auto acc = 0.0;

        const auto stratergy_type = const_state().ttl_cmd->strategy_type.strategy_type;
        if (stratergy_type == race_msgs::msg::StrategyType::CRUISE_CONTROL) {
          acc = acc_pid_.update(
            target_speed - current_speed,
            const_config().control_output_interval_sec);
        } else if (stratergy_type == race_msgs::msg::StrategyType::FOLLOW_MODE) {
          const auto speed_diff = const_state().ttl_cmd->rival_car_speed - current_speed;
          const auto gap_diff = const_state().ttl_cmd->target_gap -
            const_state().ttl_cmd->rival_car_gap;
          const auto gap_term = std::clamp(
            follow_mode_max_gap_diff_ratio_ * gap_diff,
            -follow_mode_max_speed_diff_,
            follow_mode_max_speed_diff_);
          const auto target_speed_diff = std::min(
            speed_diff - gap_term,
            target_speed - current_speed);
          acc = acc_pid_.update(target_speed_diff, const_config().control_output_interval_sec);
          state().telemetry->target_speed = target_speed_diff;
        } else {
          RCLCPP_WARN_THROTTLE(
            node().get_logger(),
            *node().get_clock(), 500,
            "Unknown stratergy type in TargetTrajectoryCommand of %u. Stopping.",
            stratergy_type);
          acc = acc_pid_.update(
            0.0 - current_speed,
            const_config().control_output_interval_sec);
        }

        // limit the stopping dcc in norminal and safe stop
        if (stop_type == race_msgs::msg::StopType::STOP_TYPE_NOMINAL ||
          stop_type == race_msgs::msg::StopType::STOP_TYPE_SAFE)
        {
          acc = std::clamp(acc, norminal_max_dcc_, acc);
        }

        // calculate lon control
        auto output = model().calc_lon_control(
          race::vehicle_model::VehicleModelState{
            current_speed,
            slip_angle_,
            const_state().path->back().curvature,
            const_state().kin_state->accel.accel.linear.x,
            const_state().kin_state->accel.accel.linear.y,
            const_state().kin_state->car_yaw_rate,
            const_state().kin_state->front_wheel_angle_rad,
            const_state().path->back().bank_angle,
            const_state().engine_report->engine_rpm,
            static_cast<size_t>(const_state().engine_report->current_gear)
          }, acc);
        output.throttle_level *= throttle_scale_;
        output.throttle_level = std::clamp(output.throttle_level, 0.0, throttle_max_);
        slip_angle_ = output.slip_angle_rad;

        state().telemetry->throttle_cmd = output.throttle_level;
        state().telemetry->brake_cmd = output.brake_psi;
        state().telemetry->target_lon_acc = acc;

        if (output.brake_psi > 0.0 && output.brake_psi > brake_deadband_psi_) {
          output.throttle_level = 0.0;
          // filter output
          output.brake_psi = brake_filter_.update(output.brake_psi);
        } else {
          // filter output
          output.throttle_level = throttle_filter_.update(output.throttle_level);
          output.brake_psi = 0.0;
        }
        // put parking brake
        if (current_speed < stopping_speed_ && target_speed < stopping_speed_) {
          RCLCPP_INFO_THROTTLE(
            node().get_logger(),
            *(node().get_clock()), 1000, "Vehicle stopped and target speed zero. Holding brake.");
          output.throttle_level = 0.0;
          output.brake_psi = parking_brake_psi_;
        }

        output_cmd.accelerator_cmd = output.throttle_level;
        output_cmd.brake_cmd = output.brake_psi;
        output_cmd.push2pass_cmd = const_state().ttl_cmd->push2pass_cmd |
          const_state().input_manual_cmd->vehicle_control_command.push2pass_cmd;

        if (test_mode_) {
          if (output_cmd.brake_cmd > 0.0) {
            output_cmd.accelerator_cmd = 0.0;
            if (const_max_brake_) {
              output_cmd.brake_cmd = max_brake_;
            } else {
              output_cmd.brake_cmd = std::min(output_cmd.brake_cmd, max_brake_);
            }
            state().telemetry->throttle_cmd = output_cmd.accelerator_cmd;
            state().telemetry->brake_cmd = output_cmd.brake_cmd;
          }
        }
      }
    } else {
      output_cmd.speed_cmd = target_speed;
      output_cmd.lon_control_type = VehicleControlCommand::LON_CONTROL_SPEED;
      output_cmd.push2pass_cmd = const_state().ttl_cmd->push2pass_cmd |
        const_state().input_manual_cmd->vehicle_control_command.push2pass_cmd;
    }
  }
};
}   // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::ModelBasedLonControl, race::RvcPlugin)
