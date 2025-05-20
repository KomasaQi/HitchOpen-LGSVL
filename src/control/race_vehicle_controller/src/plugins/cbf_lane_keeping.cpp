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

#include <cmath>
#include <memory>
#include <algorithm>
#include <cstdlib>
#include <numeric>
#include <vector>

#include "base_common/low_pass_filter.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "race_msgs/msg/engine_report.hpp"
#include "race_vehicle_controller/rvc_utils.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/hal/interface.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std::chrono_literals;

using race_msgs::msg::EngineReport;

namespace race
{

class CBFLaneKeeping : public RvcPlugin
{
public:
  // double traction_circle_cost(double ax_tire, double ay_tire, const VehicleModelState & state)
  // {
  //     const auto lat_force = model().get_config().chassis_config->total_mass * ay_tire;
  //     const auto long_force = model().get_config().chassis_config->total_mass * ax_tire;
  //     const auto front_lat_force = lat_force * model().get_config().chassis_config->cg_ratio;
  //     const auto rear_lat_force =
  //       lat_force * (1 - model().get_config().chassis_config->cg_ratio);
  //     const auto front_long_force =
  //       long_force * model().get_config().chassis_config->cg_ratio;
  //     const auto rear_long_force =
  //       long_force * (1 - model().get_config().chassis_config->cg_ratio);

  //     double max_front_lat_force = 0.0;
  //     double max_front_lon_force = 0.0;
  //     double max_rear_lat_force = 0.0;
  //     double max_rear_lon_force = 0.0;

  //     // model().calc_max_lon_lat_forces(
  //     //     state, max_front_lon_force, max_front_lat_force, max_rear_lon_force,
  //     //     max_rear_lat_force);

  //     // Cost of violation of constraints at front tire
  //     auto cost = std::max(pow(front_lat_force/max_front_lat_force,2) +
  //       pow(front_long_force/max_front_lon_force,2) - 1.,0.);

  //     // Cost of violation of constraints at rear tire
  //     cost = std::max(cost,pow(rear_lat_force/max_rear_lat_force,2) +
  //       pow(rear_long_force/max_rear_lon_force,2) - 1.);

  //     // Find max lon acc on a diamond traction circle model
  //     return 1e5 * cost;
  // }

  int get_optimal_control(
    double steer_var, double v, double theta, double theta_var, double x,
    double x_var, double curvature, double curvature_var,
    double & opt_throttle, double & opt_steering, cv::Mat & img)
  {
    double steer_ref = opt_steering;
    double throttle_ref = opt_throttle;
    double min_cost = DBL_MAX;
    (void) steer_var;
    (void) theta_var;
    (void) x_var;
    (void) curvature_var;
    double ax = 0.;
    int modified = 0;
    int r = -1;
    int c = -1;
    for (double throttle = throttle_ref / 100. - THROTTLE_RANGE;
      throttle < throttle_ref / 100. + THROTTLE_RANGE;
      throttle += (2. * THROTTLE_RANGE / THROTTLE_RES))
    {
      r += 1;
      c = -1;
      for (double steer = steer_ref - STEER_RANGE; steer < steer_ref + STEER_RANGE;
        steer += (2. * STEER_RANGE / STEER_RES))
      {
        c += 1;
        if (throttle < THROTTLE_MIN || throttle > THROTTLE_MAX) {
          if (VISUALIZE_CBF_MAP) {
            img.at<cv::Vec3b>(r, c) = {0, 0, 255};
            // img.at<cv::Vec3b>(r,c)[1] = 0;
            // img.at<cv::Vec3b>(r,c)[0] = 0;
          }
          continue;
        }
        if (steer < STEER_MIN || steer > STEER_MAX) {
          if (VISUALIZE_CBF_MAP) {
            img.at<cv::Vec3b>(r, c) = {0, 0, 255};
            // img.at<cv::Vec3b>(r,c)[1] = 0;
            // img.at<cv::Vec3b>(r,c)[0] = 0;
          }
          continue;
        }
        if (!ONLY_STEERING) {
          // double ax = model().calc_ax(
          // throttle*100., // throttle
          // VehicleModelState{ // state
          //     v, // speed_mps
          //     0.0, // slip_angle
          //     const_state().path->front().curvature, // steer_radius
          //     const_state().kin_state->accel.accel.linear.x, // lon_acc
          //     const_state().kin_state->accel.accel.linear.y, // lat_acc
          //     const_state().kin_state->car_yaw_rate, // yaw_rate
          //     const_state().kin_state->front_wheel_angle_rad, // steer_angle
          //     const_state().path->front().bank_angle, // bank_angle
          //     last_engine_msg_->engine_rpm, // engine_rpm
          //     static_cast<size_t>(last_engine_msg_->current_gear) // gear_num
          // });
        }

        // TODO(dvij): Complete these after model based longitudinal control is finalized

        // double ax_tire = model().calc_ax_tire(
        //     throttle*100.,
        //     VehicleModelState{
        //         v,
        //         0.0,
        //         const_state().path->front().curvature,
        //         const_state().kin_state->accel.accel.linear.x,
        //         const_state().kin_state->accel.accel.linear.y,
        //         const_state().kin_state->car_yaw_rate,
        //         const_state().kin_state->front_wheel_angle_rad,
        //         const_state().path->front().bank_angle,
        //         last_engine_msg_->engine_rpm,
        //         static_cast<size_t>(last_engine_msg_->current_gear)
        //     });

        // double ay_tire = model().calc_ay_tire(
        //     steer,
        //     VehicleModelState{
        //         v,
        //         0.0,
        //         const_state().path->front().curvature,
        //         const_state().kin_state->accel.accel.linear.x,
        //         const_state().kin_state->accel.accel.linear.y,
        //         const_state().kin_state->car_yaw_rate,
        //         const_state().kin_state->front_wheel_angle_rad,
        //         const_state().path->front().bank_angle,
        //         last_engine_msg_->engine_rpm,
        //         static_cast<size_t>(last_engine_msg_->current_gear)
        //     });

        // double ay_tire = model().calc_ay_tire(
        //     steer,
        //     VehicleModelState{
        //         v,
        //         0.0,
        //         const_state().path->front().curvature,
        //         const_state().kin_state->accel.accel.linear.x,
        //         const_state().kin_state->accel.accel.linear.y,
        //         const_state().kin_state->car_yaw_rate,
        //         const_state().kin_state->front_wheel_angle_rad,
        //         const_state().path->front().bank_angle,
        //         last_engine_msg_->engine_rpm,
        //         static_cast<size_t>(last_engine_msg_->current_gear)
        //     });

        double cost = std::pow(steer - steer_ref, 2) + std::pow(throttle - throttle_ref / 100., 2);
        if (LANE_KEEPING_KINEMATIC) {
          double h_left = LANE_WIDTH_L - x;
          double hd_left = -v * std::sin(theta);
          double hdd_left = -ax * std::sin(theta) - v * v * std::cos(theta) * steer / wheelbase_m +
            v * v * curvature;
          double h_right = LANE_WIDTH_R + x;
          double hd_right = v * std::sin(theta);
          double hdd_right = ax * std::sin(theta) + v * v * std::cos(theta) * steer / wheelbase_m -
            v * v * curvature;
          if (hdd_left + 2 * lambda * hd_left + lambda * lambda * h_left < 0) {
            cost += alpha * std::pow(hdd_left + 2 * lambda * hd_left + lambda * lambda * h_left, 2);
            if (VISUALIZE_CBF_MAP) {
              img.at<cv::Vec3b>(r, c) = {0, 0, 255};
            }
          }
          if (hdd_right + 2 * lambda * hd_right + lambda * lambda * h_right < 0) {
            cost += alpha * std::pow(
              hdd_right + 2 * lambda * hd_right + lambda * lambda * h_right,
              2);
            if (VISUALIZE_CBF_MAP) {
              img.at<cv::Vec3b>(r, c) = {0, 0, 255};
            }
          }
        }
        if (TRACTION_KINEMATIC) {
          double omega = v * steer / wheelbase_m;
          if (std::abs(v * omega) > AY_MAX) {
            if (VISUALIZE_CBF_MAP) {
              img.at<cv::Vec3b>(r, c) = {0, 0, 255};
            }
            cost += DYNAMIC_VIOLATION_CONST + DYNAMIC_VIOLATION_FACTOR * std::pow(
              std::abs(
                v * omega) - AY_MAX, 2);
          }
        }
        if (TRACTION_DYNAMIC) {
          double omega = const_state().kin_state->car_yaw_rate;
          if ((v * steer / wheelbase_m) > omega + K_DYN * (AY_MAX_DYN / v - omega)) {
            cost += DYNAMIC_VIOLATION_CONST + DYNAMIC_VIOLATION_FACTOR *
              std::pow((v * steer / wheelbase_m) - (omega + K_DYN * (AY_MAX_DYN / v - omega)), 2);
          }
        }

        if (cost < min_cost) {
          min_cost = cost;
          opt_steering = steer;
          opt_throttle = throttle * 100.;
        }
      }
    }
    opt_steering = output_filter_.update(opt_steering);
    if ((steer_ref - opt_steering) * (steer_ref - opt_steering) +
      (throttle_ref / 100. - opt_throttle / 100.) * (throttle_ref / 100. - opt_throttle / 100.) >
      1e-6)
    {
      modified = static_cast<int>(100 * (steer_ref - opt_steering));
      int r =
        static_cast<int>((opt_throttle / 100. - (throttle_ref / 100. - THROTTLE_RANGE)) /
        (2. * THROTTLE_RANGE / THROTTLE_RES));
      int c = static_cast<int>((opt_steering -
        (steer_ref - STEER_RANGE)) / (2. * STEER_RANGE / STEER_RES));

      if (VISUALIZE_CBF_MAP) {
        img.at<cv::Vec3b>(r, c) = {20, 20, 20};
      }
    }
    if (VISUALIZE_CBF_MAP) {
      img.at<cv::Vec3b>(THROTTLE_RES / 2, STEER_RES / 2) = {0, 0, 0};
    }
    return modified;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "cbf.lambda") {
        lambda = param.as_double();
      } else if (name == "cbf.alpha") {
        alpha = param.as_double();
      } else if (name == "cbf.steer_min") {
        STEER_MIN = param.as_double();
      } else if (name == "cbf.steer_max") {
        STEER_MAX = param.as_double();
      } else if (name == "cbf.steer_range") {
        STEER_RANGE = param.as_double();
      } else if (name == "cbf.throttle_min") {
        THROTTLE_MIN = param.as_double();
      } else if (name == "cbf.throttle_max") {
        THROTTLE_MAX = param.as_double();
      } else if (name == "cbf.throttle_range") {
        THROTTLE_RANGE = param.as_double();
      } else if (name == "cbf.lane_keeping_kinematic.lane_width_l") {
        LANE_WIDTH_L = param.as_double();
      } else if (name == "cbf.lane_keeping_kinematic.lane_width_r") {
        LANE_WIDTH_R = param.as_double();
      } else if (name == "cbf.traction_kinematic.ay_max") {
        AY_MAX = param.as_double();
      } else {
        return false;
      }
      return true;
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (name == "cbf.steer_res") {
        STEER_RES = param.as_int();
      } else if (name == "cbf.throttle_res") {
        THROTTLE_RES = param.as_int();
      } else {
        return false;
      }
      return true;
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "cbf.safeguard") {
        SAFEGUARD = param.as_bool();
      } else if (name == "cbf.only_steering") {
        ONLY_STEERING = param.as_bool();
      } else if (name == "cbf.lane_keeping_kinematic.active") {
        LANE_KEEPING_KINEMATIC = param.as_bool();
      } else if (name == "cbf.traction_kinematic.active") {
        TRACTION_KINEMATIC = param.as_bool();
      } else {
        return false;
      }
      return true;
    } else {
      return false;
    }
    return true;
  }

  bool configure() override
  {
    lon_control_type_ = 0;
    engine_sub_ = node().create_subscription<EngineReport>(
      "engine_report",
      rclcpp::SensorDataQoS(), [&](EngineReport::SharedPtr msg) {on_engine_report(msg);});
    publisher_ =
      node().create_publisher<sensor_msgs::msg::Image>("cbf_map", rclcpp::SensorDataQoS());
    x = 0.;
    theta = 0.;
    v = 1.;

    SAFEGUARD = node().declare_parameter<bool>("cbf.safeguard");
    ONLY_STEERING = node().declare_parameter<bool>("cbf.only_steering");
    lambda = node().declare_parameter<double>("cbf.lambda");
    alpha = node().declare_parameter<double>("cbf.alpha");
    STEER_RES = node().declare_parameter<int>("cbf.steer_res");
    THROTTLE_RES = node().declare_parameter<int>("cbf.throttle_res");

    STEER_MIN = node().declare_parameter<double>("cbf.steer_min");
    STEER_MAX = node().declare_parameter<double>("cbf.steer_max");
    STEER_RANGE = node().declare_parameter<double>("cbf.steer_range");
    THROTTLE_MIN = node().declare_parameter<double>("cbf.throttle_min");
    THROTTLE_MAX = node().declare_parameter<double>("cbf.throttle_max");
    THROTTLE_RANGE = node().declare_parameter<double>("cbf.throttle_range");

    LANE_KEEPING_KINEMATIC = node().declare_parameter<bool>("cbf.lane_keeping_kinematic.active");
    LANE_WIDTH_L = node().declare_parameter<double>("cbf.lane_keeping_kinematic.lane_width_l");
    LANE_WIDTH_R = node().declare_parameter<double>("cbf.lane_keeping_kinematic.lane_width_r");
    TRACTION_KINEMATIC = node().declare_parameter<bool>("cbf.traction_kinematic.active");
    AY_MAX = node().declare_parameter<double>("cbf.traction_kinematic.ay_max");
    VISUALIZE_CBF_MAP = node().declare_parameter<bool>("cbf.visualize_cbf_map");
    publisher_status = node().create_publisher<std_msgs::msg::Int8>("cbf_status", 10);

    K_DYN = node().declare_parameter<double>("cbf.k_dyn");
    AY_MAX_DYN = node().declare_parameter<double>("cbf.ay_max_dyn");
    TRACTION_DYNAMIC = node().declare_parameter<bool>("cbf.traction_dynamic");

    DYNAMIC_VIOLATION_CONST = node().declare_parameter<double>("cbf.dynamic_violation_const");
    DYNAMIC_VIOLATION_FACTOR = node().declare_parameter<double>("cbf.dynamic_violation_factor");

    img = cv::Mat(THROTTLE_RES + 1, STEER_RES + 1, CV_8UC3, cv::Scalar(0, 0, 0));
    output_filter_ = LowPassFilter(
      2.0,
      const_config().control_output_interval_sec
    );
    wheelbase_m = const_config().wheelbase_m;

    return true;
  }

  void on_trajectory_update() override
  {
    x = const_state().telemetry->lateral_error;
    theta = const_state().path->front().target_yaw;
    v = const_state().kin_state->speed_mps;
    curvature = 1. / const_state().path->front().curvature;
  }

  // TODO(dvij): Complete these after model based longitudinal control is finalized
  // double find_available_lat_acc(){

  // }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    if (!const_state().all_input_received() || !const_state().path) {
      return true;
    }
    auto message = std_msgs::msg::Int8();
    cv_bridge::CvImagePtr cv_ptr;
    if (VISUALIZE_CBF_MAP) {
      img.setTo(cv::Scalar(0, 255, 0));
    }

    // update state
    x = -const_state().telemetry->lateral_error;
    theta = -const_state().path->front().target_yaw;
    v = const_state().kin_state->speed_mps;
    curvature = 1. / const_state().path->front().curvature;

    // set goals
    const auto & stop_type = const_state().ttl_cmd->stop_type.stop_type;
    auto target_speed = const_state().path->front().target_speed;
    if (stop_type != race_msgs::msg::StopType::STOP_TYPE_NOMINAL) {
      target_speed = 0.0;
    }

    // calculate control
    if (lon_control_type_ == 0) {
      if (output_cmd.brake_cmd < 0.01) {
        auto modified = get_optimal_control(
          0., v, theta, 0., x, 0., curvature, 0.,
          output_cmd.accelerator_cmd, output_cmd.steering_cmd,
          img);
        message.data = modified;
        publisher_status->publish(message);
        if (VISUALIZE_CBF_MAP) {
          sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img)
            .toImageMsg();
          publisher_->publish(*msg);
        }
      }
      return true;
    } else {
      output_cmd.speed_cmd = target_speed;
      output_cmd.lon_control_type = VehicleControlCommand::LON_CONTROL_SPEED;
    }
    return true;
  }

private:
  void on_engine_report(EngineReport::SharedPtr msg)
  {
    last_engine_msg_ = msg;
  }
  rclcpp::Subscription<EngineReport>::SharedPtr engine_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_status;
  EngineReport::SharedPtr last_engine_msg_;
  cv::Mat img;
  bool VISUALIZE_CBF_MAP;
  double wheelbase_m;
  bool SAFEGUARD, LANE_KEEPING_KINEMATIC, TRACTION_KINEMATIC, ONLY_STEERING, TRACTION_DYNAMIC;
  double LANE_WIDTH_L, LANE_WIDTH_R, AY_MAX, STEER_MIN, STEER_MAX, STEER_RANGE, THROTTLE_MIN,
    THROTTLE_MAX, THROTTLE_RANGE, STEER_RES, THROTTLE_RES;
  double x, theta, v, curvature;
  double lambda, alpha;
  double K_DYN, AY_MAX_DYN;
  double DYNAMIC_VIOLATION_CONST, DYNAMIC_VIOLATION_FACTOR;
  LowPassFilter output_filter_;
  uint8_t lon_control_type_;
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::CBFLaneKeeping, race::RvcPlugin)
