// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_orange_flag.hpp"

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

IfOrangeFlag::IfOrangeFlag(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList IfOrangeFlag::providedPorts()
{
  return {
    BT::InputPort<const race::Flags>(
      "current_flags")
  };
}

BT::NodeStatus IfOrangeFlag::tick()
{
  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_orange_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "Orange Flag");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
