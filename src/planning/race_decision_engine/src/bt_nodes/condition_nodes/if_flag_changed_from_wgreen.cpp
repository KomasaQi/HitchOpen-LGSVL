// Copyright 2024 Moises Lopez
// FIXME: This module does not seem to be working correctly, adding some logging to test.
#include "race_decision_engine/bt_nodes/condition_nodes/if_flag_changed_from_wgreen.hpp"

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

namespace race::planning::race_decision_engine::bt::condition_nodes
{

IfFlagChangedFromWGreen::IfFlagChangedFromWGreen(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList IfFlagChangedFromWGreen::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const race::Flags>(
      "previous_flags"
    ),
    BT::InputPort<const race::Flags>(
      "current_flags")
  };
}

BT::NodeStatus IfFlagChangedFromWGreen::tick()
{
  RCLCPP_ERROR_THROTTLE(
    logger_, clock_,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms,
    "Previous is wgreen: %d, Current is wgreen: %d",
    config().blackboard->get<const race::Flags>("previous_flags").has_wgreen_flag(),
    config().blackboard->get<const race::Flags>("current_flags").has_wgreen_flag());

  if (config().blackboard->get<const race::Flags>(
      "previous_flags").has_wgreen_flag() &&
    !config().blackboard->get<const race::Flags>("current_flags").has_wgreen_flag())
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
