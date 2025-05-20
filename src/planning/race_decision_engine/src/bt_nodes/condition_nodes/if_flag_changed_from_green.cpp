// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_flag_changed_from_green.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "base_common/race_control.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace bt
{
namespace condition_nodes
{

IfFlagChangedFromGreen::IfFlagChangedFromGreen(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IfFlagChangedFromGreen::providedPorts()
{
  return {
    BT::InputPort<const race::Flags>(
      "previous_flags"
    ),
    BT::InputPort<const race::Flags>(
      "current_flags")
  };
}

BT::NodeStatus IfFlagChangedFromGreen::tick()
{
  if (config().blackboard->get<const race::Flags>(
      "previous_flags").has_green_flag() &&
    !config().blackboard->get<const race::Flags>("current_flags").has_green_flag())
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
