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


#include <math.h>
#include <iostream>
#include <vector>
#include <memory>

#include "race_vehicle_controller/plugin.hpp"

namespace race
{
using ttl::Position;

class AddSteerBias : public RvcPlugin
{
public:
  bool modify_control_command(VehicleControlCommand & output_cmd) override
  {
    output_cmd.steering_cmd -= const_config().turn_left_bias_rad;
    return true;
  }

  const char * get_plugin_name() override
  {
    return "Add Steer Bias";
  }
};

}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::AddSteerBias, race::RvcPlugin)
