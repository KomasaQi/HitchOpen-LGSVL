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
 * @file timeout.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief Check for message timeouts
 * @version 0.2
 * @date 2022-05-18
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_vehicle_controller/plugin.hpp"

using race_msgs::msg::VehicleControlCommand;

namespace race
{

struct TimeoutState
{
  bool has_timeout_ = false;
  bool has_uninitialized_clock_ = true;
  std::string timed_out_clock_names_ = "";
  std::string uninitialized_clock_names_ = "";
};

/**
 * @brief Critical Topic Timeouts
 *
 */
class Timeout : public RvcPlugin
{
public:
  typedef std::shared_ptr<Timeout> SharedPtr;

  bool configure() override
  {
    add_watch("kinematic_state", node().declare_parameter<double>("timeout.kinematic_state", 0.5));
    add_watch("ttl_command", node().declare_parameter<double>("timeout.ttl_command", 0.5));
    add_watch("manual_command", node().declare_parameter<double>("timeout.manual_command", 0.5));
    add_watch("trajectory", node().declare_parameter<double>("timeout.trajectory", 0.5));
    add_watch("output_command", node().declare_parameter<double>("timeout.output_command", 0.5));
    return true;
  }
  void on_kinematic_update() override
  {
    reset_watchdog("kinematic_state", const_state().kin_state->header.stamp);
  }
  void on_ttl_command_update() override
  {
    reset_watchdog("ttl_command", const_state().ttl_cmd->stamp);
  }
  void on_manual_command_update() override
  {
    reset_watchdog("manual_command", state().input_manual_cmd->header.stamp);
  }
  void on_trajectory_update()override
  {
    reset_watchdog("trajectory", node().now());
  }
  void on_command_output()override
  {
    reset_watchdog("output_command", const_state().output_cmd->stamp);
  }
  bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    if (check_trigger(rclcpp::Time(input_cmd.stamp))) {
      input_cmd.accelerator_cmd = 0.0;
      input_cmd.speed_cmd = 0.0;
    }
    state().telemetry->has_uninitialized_clock = has_uninitialized();
    state().telemetry->has_timeout = has_timeout();
    state().telemetry->uninitialized_clocks = state_.timed_out_clock_names_;
    state().telemetry->timeout_clocks = state_.timed_out_clock_names_;
    return true;
  }

  void add_watch(const std::string & name, const double & max_dropout_sec)
  {
    table_.insert(
      {name, TimerEntry{rclcpp::Time(
            0, 0,
            RCL_CLOCK_UNINITIALIZED), rclcpp::Duration::from_seconds(
            max_dropout_sec)}});
  }

  void reset_watchdog(const std::string & name, const rclcpp::Time & time)
  {
    last_time(table_.at(name)) = time;
  }

  bool has_timeout()
  {
    return state_.has_timeout_;
  }

  bool has_uninitialized()
  {
    return state_.has_uninitialized_clock_;
  }

  const TimeoutState & get_state()
  {
    return state_;
  }

  bool check_trigger(const rclcpp::Time & new_time)
  {
    auto triggered = false;
    state_.timed_out_clock_names_ = "";
    state_.uninitialized_clock_names_ = "";
    state_.has_uninitialized_clock_ = false;
    for (auto & timer : table_) {
      if (last_time(timer_entry(timer)).get_clock_type() == RCL_CLOCK_UNINITIALIZED) {
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *(node().get_clock()), 500, "%s: watchdog not initialized.", timer_name(timer).c_str());
        state_.uninitialized_clock_names_ += timer_name(timer) + ", ";
        triggered = true;
        state_.has_uninitialized_clock_ = true;
      } else if (new_time - last_time(timer_entry(timer)) > timeout(timer_entry(timer))) {
        RCLCPP_WARN_THROTTLE(
          node().get_logger(),
          *(node().get_clock()), 500, "%s: watchdog triggered.", timer_name(timer).c_str());
        state_.timed_out_clock_names_ += timer.first + ", ";
        triggered = true;
      }
    }
    state_.has_timeout_ = triggered;
    return triggered;
  }

  const char * get_plugin_name() override
  {
    return "Timeout Plugin";
  }

protected:
// Pair of last watchdog reset timestamp and watchdog trigger time
  typedef std::pair<rclcpp::Time, rclcpp::Duration> TimerEntry;

// Dictionary of timeout source name and timer entry
  typedef std::map<std::string, TimerEntry> TimerTable;
  typedef std::pair<const std::string, TimerEntry> TimerTableRow;

  TimerTable table_ {};
  TimeoutState state_;

  rclcpp::Time & last_time(TimerEntry & entry)
  {
    return entry.first;
  }

  rclcpp::Duration & timeout(TimerEntry & entry)
  {
    return entry.second;
  }

  const std::string & timer_name(TimerTableRow & row)
  {
    return row.first;
  }

  TimerEntry & timer_entry(TimerTableRow & row)
  {
    return row.second;
  }
};
}  // namespace race

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(race::Timeout, race::RvcPlugin)
