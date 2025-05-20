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
// Creates paths for the RVC to track

#include <math.h>
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>

#include "race_vehicle_controller/plugin.hpp"

using race::ttl::Position;

namespace race
{
class SyntheticSteeringCommand : public RvcPlugin
{
public:
  // Parameters for the sinusoidal steering command
  double sin_frequency;
  double sin_amplitude;
  bool is_active;

  rclcpp::Time start_time;
  bool initialized = false;
  bool previous_active_state = false;

  bool configure() override
  {
    sin_frequency = node().declare_parameter<double>("synthetic_steering_command.freq", 1.0);
    sin_amplitude = node().declare_parameter<double>("synthetic_steering_command.amplitude", 1.0);
    is_active = node().declare_parameter<bool>("synthetic_steering_command.activated", false);

    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    if (name == "synthetic_steering_command.freq") {
      sin_frequency = param.as_double();
    } else if (name == "synthetic_steering_command.amplitude") {
      sin_amplitude = param.as_double();
    } else if (name == "synthetic_steering_command.activated") {
      bool new_is_active = param.as_bool();

      if (new_is_active && !previous_active_state) {
        initialized = false;
      }

      is_active = new_is_active;
      previous_active_state = new_is_active;
    } else {
      return false;
    }
    return true;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    if (is_active) {
      if (!initialized) {
        start_time = node().now();
        initialized = true;
      }

      rclcpp::Time now = node().now();
      double time_seconds = (now - start_time).seconds();

      double steering_angle = sin_amplitude * std::sin(2 * M_PI * sin_frequency * time_seconds);

      output_cmd.steering_cmd = output_cmd.steering_cmd + steering_angle;
    }

    return true;
  }

  const char * get_plugin_name() override
  {
    return "SyntheticSteeringCommand";
  }
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::SyntheticSteeringCommand, race::RvcPlugin)
