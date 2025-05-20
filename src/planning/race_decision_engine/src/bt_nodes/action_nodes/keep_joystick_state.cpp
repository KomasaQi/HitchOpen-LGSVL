// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/keep_joystick_state.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine/common/input.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "race_msgs/msg/stop_type.hpp"
#include "race_msgs/msg/strategy_type.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace bt
{
namespace action_nodes
{

KeepJoystickState::KeepJoystickState(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList KeepJoystickState::providedPorts()
{
  return {
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs"),
    BT::OutputPort<bool>("auto_enabled"),
    BT::OutputPort<bool>("limit_auto_throttle"),
  };
}

BT::NodeStatus KeepJoystickState::tick()
{
  config().blackboard->set<bool>(
    "auto_enabled",
    !config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")->input_manual_cmd_msg->use_manual_cmd);
  config().blackboard->set<bool>(
    "limit_auto_throttle",
    config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")->input_manual_cmd_msg->limit_auto_throttle);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
