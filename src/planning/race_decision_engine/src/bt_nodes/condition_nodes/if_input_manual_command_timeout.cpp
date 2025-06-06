// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_input_manual_command_timeout.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

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

IfInputManualCommandTimeout::IfInputManualCommandTimeout(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList IfInputManualCommandTimeout::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")
  };
}

BT::NodeStatus IfInputManualCommandTimeout::tick()
{
  if (common::CheckTimeout(
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->manual_overrides.auto_enabled,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->timeout.manual.input_manual_command_timeout,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->timeout.autonomous.input_manual_command_timeout,
      config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
        "rde_inputs")->input_manual_cmd_diff))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms,
      "Input Manual Command Timeout timeout in %d with %f",
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->manual_overrides.auto_enabled,
      config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
        "rde_inputs")->input_manual_cmd_diff);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
