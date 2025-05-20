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
 * @file plugin.hpp
 * @author Haoru Xue (haorux@andrew.cmu.edu)
 * @brief plugin interface
 * @version 0.1
 * @date 2022-05-14
 *
 * @copyright Copyright 2022 AI Racing Tech
 *
 */

#ifndef RACE_VEHICLE_CONTROLLER__PLUGIN_HPP_
#define RACE_VEHICLE_CONTROLLER__PLUGIN_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "race_msgs/msg/vehicle_control_command.hpp"
#include "race_vehicle_controller/rvc_state.hpp"
#include "vehicle_model/vehicle_model.hpp"

namespace race
{
using race_msgs::msg::VehicleControlCommand;
class RvcPlugin
{
public:
  virtual ~RvcPlugin() {}

  /**
  * @brief Initialize plugin, saving pointers to state, config, and
  * parent node. Do not override unless needed.
  *
  * @param state Pointer to the shared state info, updated by RVC node
  * @param config Pointer to the shared config info, updated by RVC node
  * @param node Pointer to the parent node for logging, timing, etc.
  */
  virtual void initialize(
    RvcState::SharedPtr state, RvcConfig::SharedPtr config,
    race::vehicle_model::VehicleModel::SharedPtr model,
    rclcpp::Node * node)
  {
    m_state_ = state;
    m_config_ = config;
    m_model_ = model;
    m_node_ = node;
  }

  /**
   * @brief Declare ROS params, pubs, and subs, and initialize the
   * plugin here
   *
   * @return If initialization is successful
   */
  virtual bool configure() {return true;}

  /**
   * @brief Called when kinematic state is updated in RVC state.
   *
   */
  virtual void on_kinematic_update() {}

  /**
   * @brief Called when ttl command is updated in RVC state.
   *
   */
  virtual void on_ttl_command_update() {}

  /**
   * @brief Called when manual command is updated in RVC state.
   *
   */
  virtual void on_manual_command_update() {}

  /**
   * @brief Called when trajectory is updated in RVC state.
   *
   */
  virtual void on_trajectory_update() {}

  /**
   * @brief Called when wheel speed is updated in RVC state.
   *
   */
  virtual void on_wheel_speed_update() {}

  /**
   * @brief Called when engine report is updated in RVC state.
   *
   */
  virtual void on_engine_report_update() {}

  /**
   * @brief Called when a control command is outputed by RVC.
   *
   */
  virtual void on_command_output() {}

  /**
   * @brief Used by control algorithm plugin to request autonomous control command
   *
   * @param output_cmd Reference to put output command in
   * @return If computation is successful
   */
  virtual bool compute_control_command(VehicleControlCommand & output_cmd)
  {
    (void) output_cmd;
    return true;
  }

  /**
   * @brief Used by modifier plugin to change a control command in-place
   *
   * @param input_cmd Reference to the command to be modified
   * @return If modification is successful
   */
  virtual bool modify_control_command(VehicleControlCommand & input_cmd)
  {
    (void) input_cmd;
    return true;
  }

  /**
   * @brief Called when RVC detects a param change
   *
   * @param param The ROS parameter
   * @return bool if the plugin consumes that parameter (accepted as valid and updated)
   */
  virtual bool update_param(const rclcpp::Parameter & param)
  {
    (void) param;
    return false;
  }

  virtual const char * get_plugin_name()
  {
    return "default plugin name";
  }

  typedef std::shared_ptr<RvcPlugin> SharedPtr;
  typedef std::unique_ptr<RvcPlugin> UniquePtr;

protected:
  RvcState & state() {return *m_state_;}
  const RvcState & const_state() {return const_cast<const RvcState &>(state());}
  rclcpp::Node & node() {return *m_node_;}
  RvcConfig & config() {return *m_config_;}
  const RvcConfig & const_config() {return const_cast<const RvcConfig &>(config());}
  race::vehicle_model::VehicleModel & model() {return *m_model_;}
  void add_to_diagnostics(const diagnostic_msgs::msg::DiagnosticStatus & status)
  {
    m_state_->diagnostics->status.push_back(status);
  }

private:
  RvcState::SharedPtr m_state_ {};
  RvcConfig::SharedPtr m_config_{};
  race::vehicle_model::VehicleModel::SharedPtr m_model_{};
  rclcpp::Node * m_node_ {};
};
}  // namespace race
#endif  // RACE_VEHICLE_CONTROLLER__PLUGIN_HPP_
