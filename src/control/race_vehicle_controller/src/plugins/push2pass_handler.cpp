// Copyright 2024 AI Racing Tech
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

#include "race_vehicle_controller/plugin.hpp"
#include "race_msgs/msg/push2_pass_report.hpp"

namespace race
{
class Push2PassHandler : public RvcPlugin
{
public:
  bool configure() override
  {
    is_active_ = node().declare_parameter<bool>("push2passhandler.is_active", false);
    return true;
  }

  const char * get_plugin_name() override
  {
    return "Push2PassHandler";
  }

  bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    const double & thrtl_lim = const_state().push2pass_report->push2pass_active_app_limit;
    if (is_active_) {
      input_cmd.accelerator_cmd = std::clamp(input_cmd.accelerator_cmd, 0.0, thrtl_lim);
      state().telemetry->throttle_cmd = input_cmd.accelerator_cmd;
    }

    return true;
  }

  bool update_param(const rclcpp::Parameter & param) override
  {
    if (param.get_name() == "push2passhandler.is_active") {
      is_active_ = param.as_bool();
      return true;
    }

    return false;
  }

private:
  bool is_active_ {false};
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::Push2PassHandler, race::RvcPlugin)
