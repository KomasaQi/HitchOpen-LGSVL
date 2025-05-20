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

// Computes the front wheel slip angle
// Rear wheel slip angle can be computed using the vehicle velocity
// slip_angle_rear = atan(vy/vx)
// slip_angle_front = steer_front - atan((vy + l_f*yaw_dot)/vx)
// Or slip_angle_front = steer_front - atan(vy_front/vx_front)

#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <memory>

#include "race_vehicle_controller/plugin.hpp"

// #include "race_msgs/msg/rvc_telemetry.hpp"
// using race_msgs::msg::RvcTelemetry;

namespace race
{
class ComputeSlipAngle : public RvcPlugin
{
public:
  void on_kinematic_update() override
  {
    steer_angle_rad = const_state().kin_state->front_wheel_angle_rad;
    yaw_rate = const_state().kin_state->car_yaw_rate;
    auto vx = const_state().kin_state->velocity.twist.linear.x;
    auto vy = const_state().kin_state->velocity.twist.linear.y;

    slip_angle_front = (steer_angle_rad - atan((vy + l_f * yaw_rate) / vx)) * rad2deg;
    state().telemetry->slip_angle_deg = slip_angle_front;
  }

  const char * get_plugin_name() override
  {
    return "Compute Slip Angle Plugin";
  }

private:
  float steer_angle_rad;
  float yaw_rate;
  float slip_angle_front;

  // front wheel weight ratio, meant to be a constant
  const double & weight_ratio_front = model().get_config().chassis_config->cg_ratio;
  const double & L = model().get_config().chassis_config->wheel_base;   // wheel base (m)
  float l_f = weight_ratio_front * L;   // distance (x direction) from COM to front wheel (m)
  static constexpr float rad2deg = 180 / M_PI;
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::ComputeSlipAngle, race::RvcPlugin)
