// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_localization_emergency.hpp"

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
IfLocalizationEmergency::IfLocalizationEmergency(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList IfLocalizationEmergency::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")
  };
}

BT::NodeStatus IfLocalizationEmergency::tick()
{
  if (config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")->localization_state_msg->source_status_code > 0)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "Localization error code: %d",
      config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
        "rde_inputs")->localization_state_msg->source_status_code);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
