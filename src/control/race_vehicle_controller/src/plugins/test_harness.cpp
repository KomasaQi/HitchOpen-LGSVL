// Copyright 2023 Siddharth Saha
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
 * @file test_harness.cpp
 * @author Siddharth Saha (sisahawork@gmail.com)
 * @brief Test Harness Plugin
 * @version 0.1
 * @date 2023-06-26
 *
 * @copyright Copyright 2023 Siddharth Saha
 *
 */

#include <vector>
#include <memory>

#include "race_vehicle_controller/plugin.hpp"

namespace race
{

class TestHarness : public RvcPlugin
{
public:
  bool configure() override
  {
    override_throttle_ = node().declare_parameter<bool>("test_harness.override_throttle", false);
    override_brake_ = node().declare_parameter<bool>("test_harness.override_brake", false);
    override_steer_ = node().declare_parameter<bool>("test_harness.override_steer", false);

    override_throttle_value_ = node().declare_parameter<double>(
      "test_harness.override_throttle_value", 0.0);
    override_brake_value_ = node().declare_parameter<double>(
      "test_harness.override_brake_value",
      0.0);
    override_steer_value_ = node().declare_parameter<double>(
      "test_harness.override_steer_value",
      0.0);
    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    const auto & name = param.get_name();
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (name == "test_harness.override_throttle") {
        override_throttle_ = param.as_bool();
        return true;
      } else if (name == "test_harness.override_brake") {
        override_brake_ = param.as_bool();
        return true;
      } else if (name == "test_harness.override_steer") {
        override_steer_ = param.as_bool();
        return true;
      }
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "test_harness.override_throttle_value") {
        override_throttle_value_ = param.as_double();
        return true;
      } else if (name == "test_harness.override_brake_value") {
        override_brake_value_ = param.as_double();
        return true;
      } else if (name == "test_harness.override_steer_value") {
        override_steer_value_ = param.as_double();
        return true;
      }
    }
    return false;
  }

  bool compute_control_command(VehicleControlCommand & output_cmd) override
  {
    (void)output_cmd;
    return true;
  }

  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    if (override_throttle_ && output_cmd.accelerator_cmd > 0) {
      output_cmd.accelerator_cmd = override_throttle_value_;
    }
    if (override_brake_ && output_cmd.brake_cmd > 0) {
      output_cmd.brake_cmd = override_brake_value_;
      if (output_cmd.accelerator_cmd > 0) {
        output_cmd.accelerator_cmd = 0;
      }
    }
    if (override_steer_) {
      output_cmd.steering_cmd = override_steer_value_;
    }
    return true;
  }

  const char * get_plugin_name() override
  {
    return "Test Harness Plugin";
  }

protected:
  bool override_throttle_ {false};
  bool override_brake_ {false};
  bool override_steer_ {false};

  double override_throttle_value_ {0.0};
  double override_brake_value_ {0.0};
  double override_steer_value_ {0.0};
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::TestHarness, race::RvcPlugin)
