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
#include "std_msgs/msg/float64.hpp"

namespace race
{
class ModelBasedLatControl : public RvcPlugin
{
public:
  bool configure() override
  {
    acc_pid_ = PidController(
      "acc_pid", PidCoefficients{
        node().declare_parameter<double>("model_lat_control.acc_pid.kp", 1.0),
        node().declare_parameter<double>("model_lat_control.acc_pid.ki", 0.0),
        node().declare_parameter<double>("model_lat_control.acc_pid.kd", 0.2),
        node().declare_parameter<double>("model_lat_control.acc_pid.min_cmd", -1.0),
        node().declare_parameter<double>("model_lat_control.acc_pid.max_cmd", 1.0),
        node().declare_parameter<double>("model_lat_control.acc_pid.min_i", -0.5),
        node().declare_parameter<double>("model_lat_control.acc_pid.max_i", 0.5)});
    const auto output_cutoff_freq = node().declare_parameter<double>(
      "model_lat_control.output_cutoff_freq", 3.0);
    stopping_speed_ =
      node().declare_parameter<double>("model_lat_control.stopping_speed", 2.0);
    steering_bias =
      node().declare_parameter<double>("model_lat_control.steering_bias_deg", 0.0) * M_PI / 180.0;
    steering_filter_ = LowPassFilter(
      output_cutoff_freq,
      const_config().control_output_interval_sec
    );
    slip_pub_ = node().create_publisher<std_msgs::msg::Float64>("slip_angle", rclcpp::QoS{1});
    return true;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (!const_state().all_input_received() || !const_state().path) {
      return true;
    }

    // pure pursuit
    auto lookahead_pt = const_state().path->back().location;
    auto lookahead_angle = atan2(lookahead_pt.y, lookahead_pt.x);
    auto lookahead_distance = sqrt(pow(lookahead_pt.y, 2) + pow(lookahead_pt.x, 2));
    auto steer_rad = atan(
      (2 * const_config().wheelbase_m * sin(
        lookahead_angle)) / lookahead_distance);
    auto curvature = 2 * lookahead_pt.y / pow(lookahead_distance, 2);
    curvature = 1.0 / curvature;
    // convert to cg curvature
    curvature = sqrt(pow(curvature, 2) + pow(const_config().wheelbase_m * 0.5, 2));

    // consult model for slip angle
    auto & current_speed = const_state().kin_state->speed_mps;
    auto & current_yaw_rate = const_state().kin_state->car_yaw_rate;
    if (current_speed < stopping_speed_) {
      acc_pid_.reset_integral_error(0.0);
      current_speed = stopping_speed_;
    }
    const auto target_yaw_rate = const_state().kin_state->speed_mps / curvature;
    acc_pid_.update(
      target_yaw_rate - current_yaw_rate,
      const_config().control_output_interval_sec);
    const auto target_lat_acc = (target_yaw_rate) * current_speed;
    const auto lat_control_output = model().calc_lat_control(
      race::vehicle_model::VehicleModelState{
        current_speed,
        slip_angle_,
        curvature,
        const_state().kin_state->accel.accel.linear.x,
        const_state().kin_state->accel.accel.linear.y,
        current_yaw_rate,
        const_state().kin_state->front_wheel_angle_rad,
        const_state().path->front().bank_angle,
        0.0,
        0
      }, target_lat_acc);
    slip_angle_ = lat_control_output.cg_slip_angle_rad;
    output_cmd.steering_cmd = steer_rad +
      (lat_control_output.target_front_slip_angle_rad - lat_control_output.front_slip_angle_rad) +
      steering_bias;
    output_cmd.steering_cmd = steering_filter_.update(output_cmd.steering_cmd);
    output_cmd.steering_cmd = std::clamp(
      output_cmd.steering_cmd,
      -const_config().max_front_wheel_angle_rad, const_config().max_front_wheel_angle_rad);
    state().telemetry->steering_cmd = output_cmd.steering_cmd;
    state().telemetry->slip_angle_deg = slip_angle_ * 180.0 / M_PI;
    std_msgs::msg::Float64 slip_msg;
    slip_msg.data = slip_angle_;
    slip_pub_->publish(slip_msg);
    return true;
  }

private:
  PidController acc_pid_;
  double slip_angle_ = 0.0;
  double stopping_speed_;
  double steering_bias;
  LowPassFilter steering_filter_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_pub_ {};
};
}   // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::ModelBasedLatControl, race::RvcPlugin)
