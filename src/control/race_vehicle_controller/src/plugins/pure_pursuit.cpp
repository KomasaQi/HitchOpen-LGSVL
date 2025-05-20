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
 * @file pure_pursuit.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief pure pursuit controller
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

#include "ttl.hpp"
#include "race_vehicle_controller/plugin.hpp"
#include "base_common/low_pass_filter.hpp"
#include "base_common/rate_limiter.hpp"

namespace race
{
using ttl::Position;

struct PurePursuitOutput
{
  double steer_rad;
  double target_curvature;
};

class PurePursuitController : public RvcPlugin
{
public:
  bool configure() override
  {
    as_backup_ = node().declare_parameter("pure_pursuit.as_backup", false);
    cutoff_freq = node().declare_parameter<double>("pure_pursuit.output_cutoff_freq", 2.0);
    output_filter_ = LowPassFilter(
      cutoff_freq,
      const_config().control_output_interval_sec
    );
    lookahead_x_filter_ = RateLimiter(
      node().declare_parameter<double>("pure_pursuit.lookahead_rate_limit", 100.0),
      const_config().control_output_interval_sec
    );
    lookahead_y_filter_ = RateLimiter(
      node().get_parameter("pure_pursuit.lookahead_rate_limit").as_double(),
      const_config().control_output_interval_sec
    );
    use_vel_yaw_conversion_ = node().declare_parameter<bool>(
      "pure_pursuit.use_vel_yaw_conversion",
      false);
    lat_error_thres = node().declare_parameter<double>("pure_pursuit.lat_error_thres", 0.1);
    bank_angle_const_ = node().declare_parameter<double>("pure_pursuit.bank_angle_const", 0.0);
    return true;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    if (abs(const_state().telemetry->lateral_error) < lat_error_thres) {
      output_filter_.reconfigureFilter(
        const_config().control_output_interval_sec,
        cutoff_freq * abs(const_state().telemetry->lateral_error) / lat_error_thres);
    } else {
      output_filter_.reconfigureFilter(
        const_config().control_output_interval_sec,
        cutoff_freq);
    }
    if (!as_backup_) {
      const auto steering_output = step();
      output_cmd.steering_cmd = steering_output.steer_rad;
      output_cmd.steering_cmd = output_filter_.update(output_cmd.steering_cmd);
      state().telemetry->steering_cmd = output_cmd.steering_cmd;
    }
    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    const auto & type = param.get_type();
    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "pure_pursuit.bank_angle_const") {
        bank_angle_const_ = param.as_double();
        return true;
      }
      if (name == "pure_pursuit.output_cutoff_freq") {
        cutoff_freq = param.as_double();
        return true;
      }
      if (name == "pure_pursuit.lat_error_thres") {
        lat_error_thres = param.as_double();
        return true;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "pure_pursuit.use_vel_yaw_conversion") {
        use_vel_yaw_conversion_ = param.as_bool();
        return true;
      }
    }
    return false;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    if (as_backup_ && isnan(output_cmd.steering_cmd)) {
      RCLCPP_WARN_THROTTLE(
        node().get_logger(),
        *(node().get_clock()), 1000, "NaN steering detected. Pure Pursuit overriding.");
      const auto steering_output = step();
      output_cmd.steering_cmd = steering_output.steer_rad;
      output_cmd.steering_cmd = output_filter_.update(output_cmd.steering_cmd);
      state().telemetry->steering_cmd = output_cmd.steering_cmd;
    }
    return true;
  }

  PurePursuitOutput step()
  {
    if (!const_state().all_input_received() || !const_state().path) {
      return PurePursuitOutput{0.0, 0.0};
    }
    auto lookahead_pt = const_state().path->back().location;
    lookahead_pt.x = lookahead_x_filter_.update(lookahead_pt.x);
    lookahead_pt.y = lookahead_y_filter_.update(lookahead_pt.y);
    auto lookahead_angle = atan2(lookahead_pt.y, lookahead_pt.x);
    if (use_vel_yaw_conversion_) {
      lookahead_angle -= state().kin_state->velocity_yaw - state().kin_state->car_yaw;
    }
    lookahead_angle += bank_angle_const_ * const_state().path->front().bank_angle;
    auto lookahead_distance = sqrt(pow(lookahead_pt.y, 2) + pow(lookahead_pt.x, 2));
    auto steer_rad = atan(
      (2 * const_config().wheelbase_m * sin(
        lookahead_angle)) / lookahead_distance);
    auto curvature = 2 * lookahead_pt.y / pow(lookahead_distance, 2);

    return PurePursuitOutput{std::clamp(
        steer_rad, -const_config().max_front_wheel_angle_rad,
        const_config().max_front_wheel_angle_rad), curvature};
  }

  const char * get_plugin_name() override
  {
    return "Pure Pursuit Plugin";
  }

protected:
  bool as_backup_ {false};
  double cutoff_freq {2.0};
  LowPassFilter output_filter_;
  RateLimiter lookahead_x_filter_;
  RateLimiter lookahead_y_filter_;
  bool use_vel_yaw_conversion_ {false};
  double bank_angle_const_ {0.0};
  double lat_error_thres {0.1};
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::PurePursuitController, race::RvcPlugin)
