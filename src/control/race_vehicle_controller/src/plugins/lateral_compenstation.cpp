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
//
// Defines the lateral compenstation plugin for the RVC

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>

#include "ttl.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "race_vehicle_controller/external/pid_controller.hpp"
#include "race_vehicle_controller/rvc_utils.hpp"
#include "std_msgs/msg/float64.hpp"

namespace race
{
using ttl::Position;

class LateralCompensation : public RvcPlugin
{
public:
  bool configure() override
  {
    lateral_pid_ = std::make_unique<PidController>(
      "lateral_pid", PidCoefficients{
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.kp", 2.0),
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.ki", 0.1),
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.kd", 0.1),
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.min_cmd", 0.0),
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.max_cmd", 50.0),
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.min_i", -10.0),
        node().declare_parameter<double>("lateral_compenstation.lateral_pid.max_i", 10.0),
      });
    dt = node().declare_parameter<double>("lateral_compenstation.step_interval_sec", 0.01);
    min_speed = node().declare_parameter<double>("lateral_compenstation.min_speed", 1.0);
    max_slip_angle_rad = node().declare_parameter<double>(
      "lateral_compenstation.max_slip_angle_deg", 8.0) * M_PI / 180.0;
    ov_ud_lookup_speed = node().declare_parameter(
      "lateral_compenstation.ov_ud_steer_lookup.speed",
      std::vector<double>{});
    ov_ud_lookup_ratio = node().declare_parameter(
      "lateral_compenstation.ov_ud_steer_lookup.ratio",
      std::vector<double>{});
    ov_ud_factor = node().declare_parameter<double>("lateral_compenstation.ov_ud_factor", 0.0);
    is_learnable_ov_ud = node().declare_parameter<bool>(
      "lateral_compenstation.is_learnable_ov_ud",
      false);
    learnable_ov_ud = 0.0;
    learnable_ov_ud_min = node().declare_parameter<double>(
      "lateral_compenstation.learnable_ov_ud_min", -0.1);
    learnable_ov_ud_max = node().declare_parameter<double>(
      "lateral_compenstation.learnable_ov_ud_max", 0.1);
    learning_rate = node().declare_parameter<double>("lateral_compenstation.learning_rate", 0.1);
    learnable_ov_ud_publisher_ = node().create_publisher<std_msgs::msg::Float64>(
      "/lateral_comp/learnable_ov_ud", rclcpp::SensorDataQoS());
    steer_bias = node().declare_parameter<double>("lateral_compenstation.steer_bias", -0.013);
    min_w = node().declare_parameter<double>("lateral_compenstation.min_w", 0.1);
    min_vx = node().declare_parameter<double>("lateral_compenstation.min_vx", 0.1);
    buffer_size = node().declare_parameter<int>("lateral_compenstation.buffer_size", 100);
    enabled_ = node().declare_parameter<bool>("lateral_compenstation.enabled", true);
    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    if (name == "lateral_compenstation.lateral_pid.kp") {
      return lateral_pid_->try_update_param("kp", param);
    } else if (name == "lateral_compenstation.lateral_pid.ki") {
      return lateral_pid_->try_update_param("ki", param);
    } else if (name == "lateral_compenstation.lateral_pid.kd") {
      return lateral_pid_->try_update_param("kd", param);
    } else if (name == "lateral_compenstation.lateral_pid.min_cmd") {
      return lateral_pid_->try_update_param("min_cmd", param);
    } else if (name == "lateral_compenstation.lateral_pid.max_cmd") {
      return lateral_pid_->try_update_param("max_cmd", param);
    } else if (name == "lateral_compenstation.lateral_pid.min_i") {
      return lateral_pid_->try_update_param("min_i", param);
    } else if (name == "lateral_compenstation.lateral_pid.max_i") {
      return lateral_pid_->try_update_param("max_i", param);
    } else if (name == "lateral_compenstation.min_speed") {
      min_speed = param.as_double();
    } else if (name == "lateral_compenstation.ov_ud_factor") {
      ov_ud_factor = param.as_double();
    } else if (name == "lateral_compenstation.learnable_ov_ud_min") {
      learnable_ov_ud_min = param.as_double();
    } else if (name == "lateral_compenstation.learnable_ov_ud_max") {
      learnable_ov_ud_max = param.as_double();
    } else if (name == "lateral_compenstation.steer_bias") {
      steer_bias = param.as_double();
    } else if (name == "lateral_compenstation.min_w") {
      min_w = param.as_double();
    } else if (name == "lateral_compenstation.min_vx") {
      min_vx = param.as_double();
    } else if (name == "lateral_compenstation.buffer_size") {
      buffer_size = param.as_int();
    } else if (name == "lateral_compenstation.learning_rate") {
      learning_rate = param.as_double();
    } else if (name == "lateral_compenstation.is_learnable_ov_ud") {
      is_learnable_ov_ud = param.as_bool();
    } else if (name == "lateral_compenstation.enabled") {
      enabled_ = param.as_bool();
    } else if (name == "lateral_compenstation.ov_ud_steer_lookup.speed") {
      ov_ud_lookup_speed = param.as_double_array();
    } else if (name == "lateral_compenstation.ov_ud_steer_lookup.ratio") {
      ov_ud_lookup_ratio = param.as_double_array();
    } else {
      return false;
    }
    return true;
  }

  double compensate_lateral_error(const double & lateral_error)
  {
    auto adjustment = lateral_pid_->update(lateral_error, dt);
    if (const_state().kin_state->speed_mps < min_speed) {
      lateral_pid_->reset_integral_error(0.0);
    }
    return adjustment;
  }

  double compensate_ov_ud_steer(const double & steering_rad)
  {
    auto speed = const_state().kin_state->speed_mps;
    if (speed < 0.0 || isnan(speed)) {  // check for stationary / nan value
      speed = 0.0;
    }
    const auto adj_ratio = utils::interpolate(ov_ud_lookup_speed, ov_ud_lookup_ratio, speed, true);
    if (is_learnable_ov_ud) {
      return ((1.0 - (adj_ratio * ov_ud_factor)) + learnable_ov_ud) * (steering_rad);
    }
    return (1.0 - adj_ratio * ov_ud_factor) * (steering_rad);
  }

  double limit_slip_angle(const double & steering_rad)
  {
    if (const_state().kin_state->speed_mps < min_speed) {
      return steering_rad;
    }
    auto velocity_heading_local = const_state().kin_state->velocity_yaw -
      const_state().kin_state->car_yaw;
    if (velocity_heading_local > M_PI) {
      velocity_heading_local = -2.0 * M_PI + velocity_heading_local;
    } else if (velocity_heading_local < -1.0 * M_PI) {
      velocity_heading_local = 2.0 * M_PI + velocity_heading_local;
    }
    if (steering_rad < velocity_heading_local - max_slip_angle_rad ||
      steering_rad > velocity_heading_local + max_slip_angle_rad)
    {
      RCLCPP_INFO_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 1000, "Limiting slip angle");
    }
    return std::clamp(
      steering_rad, velocity_heading_local + max_slip_angle_rad,
      velocity_heading_local - max_slip_angle_rad);
  }

  bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    if (!enabled_) {
      return true;
    }

    if (!const_state().telemetry->mpc_enabled) {
      return true;
    }

    if (true) {
      double grad_ov_ud = 0.0;
      for (int i = 0; i < curr_ws.size(); i++) {
        double curr_w = curr_ws[i];
        double curr_steer = curr_steers[i];
        double curr_vx = curr_vxs[i];
        // auto speed = const_state().kin_state->speed_mps;
        if (curr_vx < 0.0 || isnan(curr_vx)) {  // check for stationary / nan value
          curr_vx = 0.0;
        }
        const auto adj_ratio = utils::interpolate(
          ov_ud_lookup_speed, ov_ud_lookup_ratio, curr_vx,
          true);

        double curr_w_ = (1 - adj_ratio + learnable_ov_ud) * curr_w;
        double expected_w = tan(curr_steer) * curr_vx / const_config().wheelbase_m;
        grad_ov_ud += 2 * (curr_w_ - expected_w) * curr_w;
      }
      learnable_ov_ud -= grad_ov_ud * learning_rate / std::max(1, static_cast<int>(curr_ws.size()));
      learnable_ov_ud = std::clamp(learnable_ov_ud, learnable_ov_ud_min, learnable_ov_ud_max);
      std_msgs::msg::Float64 learnable_msg;
      learnable_msg.data = learnable_ov_ud;
      learnable_ov_ud_publisher_->publish(learnable_msg);
      double w = const_state().kin_state->car_yaw_rate;
      double vx = const_state().kin_state->speed_mps;
      double steer = const_state().kin_state->front_wheel_angle_rad - steer_bias;
      if (abs(w) > min_w && vx > min_vx) {
        curr_ws.push_back(w);
        curr_steers.push_back(steer);
        curr_vxs.push_back(vx);
      }
      if (curr_ws.size() > buffer_size) {
        curr_ws.erase(curr_ws.begin());
        curr_steers.erase(curr_steers.begin());
        curr_vxs.erase(curr_vxs.begin());
      }
    }

    input_cmd.steering_cmd += compensate_lateral_error(const_state().telemetry->lateral_error);
    input_cmd.steering_cmd = compensate_ov_ud_steer(input_cmd.steering_cmd);

    return true;
  }

  const char * get_plugin_name() override
  {
    return "Lateral Compensation Plugin";
  }

private:
  std::unique_ptr<PidController> lateral_pid_;
  double dt = 0.01;
  double min_speed = 2.0;
  double last_compenstation = 0.0;
  double max_slip_angle_rad = 0.0;
  std::vector<double> ov_ud_lookup_speed {};
  std::vector<double> ov_ud_lookup_ratio {};
  double ov_ud_factor = 0.0;
  bool is_learnable_ov_ud = false;
  std::vector<double> curr_ws {};
  std::vector<double> curr_steers {};
  std::vector<double> curr_vxs {};
  double learning_rate = 0.1;
  double learnable_ov_ud = 0.0;
  double learnable_ov_ud_min = 0.0;
  double learnable_ov_ud_max = 0.0;
  double steer_bias = 0.0;
  double min_w = 0.1;
  double min_vx = 0.1;
  int buffer_size = 100;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr learnable_ov_ud_publisher_;
  bool enabled_ = true;
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::LateralCompensation, race::RvcPlugin)
