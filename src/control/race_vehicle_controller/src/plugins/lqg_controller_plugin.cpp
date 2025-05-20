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
 * @file lqg_controller_plugin.hpp
 * @author Dvij Kalaria (dkalaria@andrew.cmu.edu)
 * @brief LQG Controller
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

#include "Eigen/Core"

#include "race_msgs/srv/update_vehicle_model.hpp"
#include "race_msgs/msg/lqg_telemetry.hpp"

#include "ttl.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "lqg_controller/lqg_controller.hpp"
#include "lqg_controller/dynamic_bicycle_model.hpp"
#include "race_vehicle_controller/rvc_utils.hpp"

namespace race
{
// using namespace std::literals::chrono_literals;
using ttl::Position;
// using race_msgs::srv::UpdateVehicleModel;

class LqgControllerPlugin : public RvcPlugin
{
public:
  LqgControllerConfig::SharedPtr lqg_config = std::make_shared<LqgControllerConfig>();
  LkfCalculatorConfig::SharedPtr lkf_config = std::make_shared<LkfCalculatorConfig>();
  LqrCalculatorConfig::SharedPtr lqr_config = std::make_shared<LqrCalculatorConfig>();
  DynamicBicycleModelConfig::SharedPtr car_model_config =
    std::make_shared<DynamicBicycleModelConfig>();

  int radius_from_trajectory_factor;

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "lqr_config.are_error_thresh") {
        lqr_config->are_error_thresh = param.as_double();
      } else if (name == "lqr_config.state_performance.l_factor") {
        lqr_config->state_performance.l_factor = param.as_double();
        controller_->recalculate_params();
      } else if (name == "lqr_config.state_performance.l_min") {
        lqr_config->state_performance.l_min = param.as_double();
        controller_->recalculate_params();
      } else if (name == "lqr_config.state_performance.l_const") {
        lqr_config->state_performance.l_const = param.as_double();
        controller_->recalculate_params();
      } else if (name == "lqr_config.state_performance.rc_1_val") {
        lqr_config->state_performance.rc_1_val = param.as_double();
        controller_->recalculate_params();
      } else if (name == "lqg_config.max_steer_rad") {
        lqg_config->max_steer_rad = param.as_double();
      } else if (name == "lkf_config.model_error.qf_1") {
        lkf_config->model_error.qf_1 = param.as_double();
      } else if (name == "lkf_config.model_error.qf_2") {
        lkf_config->model_error.qf_2 = param.as_double();
      } else if (name == "lkf_config.model_error.qf_3") {
        lkf_config->model_error.qf_3 = param.as_double();
      } else if (name == "lkf_config.model_error.qf_4") {
        lkf_config->model_error.qf_4 = param.as_double();
      } else if (name == "lkf_config.measurement_error.rf_1") {
        lkf_config->measurement_error.rf_1 = param.as_double();
      } else if (name == "lkf_config.measurement_error.rf_2") {
        lkf_config->measurement_error.rf_2 = param.as_double();
      } else if (name == "lkf_config.measurement_error.rf_3") {
        lkf_config->measurement_error.rf_3 = param.as_double();
      } else if (name == "lkf_config.measurement_error.rf_4") {
        lkf_config->measurement_error.rf_4 = param.as_double();
      } else {
        return true;
      }
      return true;
    }
    return true;
  }

  bool configure() override
  {
    output_filter_ = LowPassFilter(2.0, const_config().control_output_interval_sec);
    curv_filter_ = LowPassFilter(2.0, const_config().control_output_interval_sec);

    // cout << "started" << endl;
    lqr_config->are_max_iter = node().declare_parameter<int>("lqr_config.are_max_iter");
    lqr_config->are_error_thresh = node().declare_parameter<double>("lqr_config.are_error_thresh");
    lqr_config->state_performance.l_factor = node().declare_parameter<double>(
      "lqr_config.state_performance.l_factor");
    lqr_config->state_performance.l_min = node().declare_parameter<double>(
      "lqr_config.state_performance.l_min");
    lqr_config->state_performance.l_const = node().declare_parameter<double>(
      "lqr_config.state_performance.l_const");

    lqr_config->state_performance.rc_1_val = node().declare_parameter<double>(
      "lqr_config.state_performance.rc_1_val");
    // cout << "set" << endl;
    lqr_config->Ts = node().declare_parameter<double>("lqg_config.ts");
    lqr_config->Vmin = node().declare_parameter<double>("lqg_config.min_speed_mps");
    lqr_config->Vmax = node().declare_parameter<double>("lqg_config.max_speed_mps");

    lqg_config->ts = lqr_config->Ts;
    lqg_config->min_speed_mps = lqr_config->Vmin;
    lqg_config->max_speed_mps = lqr_config->Vmax;
    lqg_config->scale_factor_x = node().declare_parameter<double>("lqg_config.scale_factor.x");
    lqg_config->scale_factor_theta = node().declare_parameter<double>(
      "lqg_config.scale_factor.theta");
    lqg_config->steady_state_error_correction_multiplier = node().declare_parameter<double>(
      "lqg_config.steady_state_error_correction_multiplier");
    lqg_config->max_steer_rad = node().declare_parameter<double>("lqg_config.max_steer_rad");
    lqg_config->LEN = node().declare_parameter<double>("lqg_model.L");
    lqg_config->use_pure_pursuit_params = node().declare_parameter<bool>(
      "lqg_config.use_pure_pursuit_params");
    lqg_config->use_differential_state_params = node().declare_parameter<bool>(
      "lqg_config.use_differential_state_params");
    lqg_config->line_error_threshold = node().declare_parameter<double>(
      "lqg_config.line_error_threshold");
    lqg_config->initial_state.x1 = node().declare_parameter<double>("lqg_config.initial_state.x1");
    lqg_config->initial_state.x2 = node().declare_parameter<double>("lqg_config.initial_state.x2");
    lqg_config->initial_state.x3 = node().declare_parameter<double>("lqg_config.initial_state.x3");
    lqg_config->initial_state.x4 = node().declare_parameter<double>("lqg_config.initial_state.x4");
    lqg_config->initial_error_cov.p0_1 = node().declare_parameter<double>(
      "lqg_config.initial_error_cov.p0_1");
    lqg_config->initial_error_cov.p0_2 = node().declare_parameter<double>(
      "lqg_config.initial_error_cov.p0_2");
    lqg_config->initial_error_cov.p0_3 = node().declare_parameter<double>(
      "lqg_config.initial_error_cov.p0_3");
    lqg_config->initial_error_cov.p0_4 = node().declare_parameter<double>(
      "lqg_config.initial_error_cov.p0_4");

    lkf_config->model_error.qf_1 = node().declare_parameter<double>("lkf_config.model_error.qf_1");
    lkf_config->model_error.qf_2 = node().declare_parameter<double>("lkf_config.model_error.qf_2");
    lkf_config->model_error.qf_3 = node().declare_parameter<double>("lkf_config.model_error.qf_3");
    lkf_config->model_error.qf_4 = node().declare_parameter<double>("lkf_config.model_error.qf_4");
    lkf_config->measurement_error.rf_1 = node().declare_parameter<double>(
      "lkf_config.measurement_error.rf_1");
    lkf_config->measurement_error.rf_2 = node().declare_parameter<double>(
      "lkf_config.measurement_error.rf_2");
    lkf_config->measurement_error.rf_3 = node().declare_parameter<double>(
      "lkf_config.measurement_error.rf_3");
    lkf_config->measurement_error.rf_4 = node().declare_parameter<double>(
      "lkf_config.measurement_error.rf_4");

    car_model_config->m = node().declare_parameter<double>("lqg_model.m");
    car_model_config->lf = node().declare_parameter<double>("lqg_model.lf");
    car_model_config->lr = node().declare_parameter<double>("lqg_model.lr");
    car_model_config->cf = node().declare_parameter<double>("lqg_model.cf");
    car_model_config->cr = node().declare_parameter<double>("lqg_model.cr");
    car_model_config->iz = node().declare_parameter<double>("lqg_model.iz");
    car_model_config->ts = node().declare_parameter<double>("lqg_model.ts");
    radius_from_trajectory_factor = node().declare_parameter<int>(
      "lqg_model.radius_from_traj_factor");
    lqg_state_ = std::make_shared<LqgState>();
    controller_ = std::make_shared<LqgController>(lqg_config, lqr_config, lkf_config);
    car_model = std::make_shared<DynamicBicycleModel>(car_model_config);
    lqg_telemetry_publisher_ = node().create_publisher<race_msgs::msg::LqgTelemetry>(
      "lqg_telemetry", rclcpp::SensorDataQoS());

    return true;
  }


  void on_kinematic_update() override
  {
    auto speed = output_filter_.update(const_state().kin_state->speed_mps);
    controller_->set_transformation(
      const_state().kin_state->pose.pose.position.x,
      const_state().kin_state->pose.pose.position.y, const_state().kin_state->car_yaw);
    controller_->set_speed(speed);

    if (speed < lqr_config->Vmin) {
      speed = lqr_config->Vmin;
    }
    double avg_curvature = 0.;
    if (traj_initialized_) {
      for (size_t i = 0; i < const_state().path->size(); i++) {
        avg_curvature += 1. / const_state().path->at(i).curvature;
      }
      avg_curvature /= const_state().path->size();
      avg_curvature = 1. / avg_curvature;
    }
    auto new_curvature = 0.;
    auto bank_angle = 0.;
    if (traj_initialized_) {
      auto path_size = const_state().path->size();
      new_curvature = curv_filter_.update(
        getCircleCurvature(
          const_state().path->at(0).location.x,
          const_state().path->at(0).location.y, const_state().path->at(
            path_size / (2 * radius_from_trajectory_factor)).location.x,
          const_state().path->at(path_size / (2 * radius_from_trajectory_factor)).location.y,
          const_state().path->at(path_size / radius_from_trajectory_factor).location.x,
          const_state().path->at(
            path_size / radius_from_trajectory_factor).location.y));
      bank_angle = const_state().path->front().bank_angle;
    }

    car_model->create_car_model(speed, new_curvature, bank_angle);

    model_initialized_ = true;
    controller_->set_model(car_model->A, car_model->B, car_model->C, car_model->D);
  }

  void on_trajectory_update() override
  {
    double curr_time = node().now().nanoseconds() / 1e9;
    controller_->set_cross_track_error(const_state().telemetry->lateral_error, curr_time);
    controller_->set_path(const_state().path, curr_time);
    traj_initialized_ = true;
  }

  void on_manual_command_update() override
  {
    joy_received_ = true;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    // cout << "Before step " << model_initialized_ << " " << traj_initialized_ << endl;
    if (!const_state().all_input_received() || !const_state().path || !model_initialized_) {
      return false;
    }
    if (model_initialized_ && traj_initialized_) {
      auto steering = controller_->lqg_step(0.0);
      output_cmd.steering_cmd = steering;
      // output_cmd.steering_cmd = output_filter_.update(steering);
      if (joy_received_) {
        lqg_telem_msg_.input_control_cmd = const_state().input_manual_cmd->vehicle_control_command;
        controller_->get_state(lqg_state_);

        lqg_telem_msg_.lqg_delta = lqg_state_->lqg_delta;
        lqg_telem_msg_.d_ff = lqg_state_->d_ff;
        lqg_telem_msg_.lqg_speed = lqg_state_->lqg_speed;
        lqg_telem_msg_.lqg_e_cg = lqg_state_->lqg_e_cg;
        lqg_telem_msg_.lqg_e_cg_dot = lqg_state_->lqg_e_cg_dot;
        lqg_telem_msg_.lqg_theta_e = lqg_state_->lqg_theta_e;
        lqg_telem_msg_.lqg_theta_e_dot = lqg_state_->lqg_theta_e_dot;
        lqg_telem_msg_.future_curvature = lqg_state_->future_curvature;
        lqg_telem_msg_.lqg_e_cg_hat = lqg_state_->lqg_e_cg_hat;
        lqg_telem_msg_.lqg_e_cg_dot_hat = lqg_state_->lqg_e_cg_dot_hat;
        lqg_telem_msg_.lqg_theta_e_hat = lqg_state_->lqg_theta_e_hat;
        lqg_telem_msg_.lqg_theta_e_dot_hat = lqg_state_->lqg_theta_e_dot_hat;
        lqg_telem_msg_.yaw = lqg_state_->yaw;
        lqg_telem_msg_.yaw_rate = lqg_state_->yaw_rate;
        lqg_telem_msg_.vx = lqg_state_->vx;
        lqg_telem_msg_.vy = lqg_state_->vy;
        lqg_telemetry_publisher_->publish(lqg_telem_msg_);
      }
    }
    return true;
  }

  double getCircleCurvature(double x1, double y1, double x2, double y2, double x3, double y3)
  {
    // Calculate the lengths of the sides of the triangle formed by the points
    double a = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    double b = sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
    double c = sqrt((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3));

    // Calculate the semi-perimeter of the triangle
    double dir1x = (x2 - x1);
    double dir1y = (y2 - y1);
    double dir2x = (x3 - x2);
    double dir2y = (y3 - y2);
    double cp = dir1x * dir2y - dir1y * dir2x;

    // Calculate the semi-perimeter of the triangle
    double s = (a + b + c) / 2;

    // Calculate the area of the triangle using Heron's formula
    double A = sqrt(s * (s - a) * (s - b) * (s - c));

    // Calculate the radius of the circle using the formula
    // r = A / s
    // double r = (a * b * c) / (4. * A);
    double r = (4. * A) / (a * b * c);
    if (cp > 0.) {return r;} else {return -r;}
  }

  const char * get_plugin_name() override
  {
    return "LQG Controller Plugin";
  }

private:
  // rclcpp::Client<UpdateVehicleModel>::SharedPtr model_client_;
  DynamicBicycleModel::SharedPtr car_model;
  rclcpp::Publisher<race_msgs::msg::LqgTelemetry>::SharedPtr lqg_telemetry_publisher_;
  race_msgs::msg::LqgTelemetry lqg_telem_msg_;
  LqgController::SharedPtr controller_;
  LqgState::SharedPtr lqg_state_;
  bool model_initialized_ = false;
  bool traj_initialized_ = false;
  bool joy_received_ = false;
  LowPassFilter output_filter_;
  LowPassFilter curv_filter_;
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::LqgControllerPlugin, race::RvcPlugin)
