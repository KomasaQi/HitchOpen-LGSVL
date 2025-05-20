// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/action_nodes/set_to_pitting.hpp"

#include <memory>
#include <set>
#include <string>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace race::planning::race_decision_engine::bt::action_nodes
{
SetToPitting::SetToPitting(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock) {}      // SetToPitting

BT::PortsList SetToPitting::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<bool>("target_value"),
    BT::OutputPort<bool>("is_pitting"),
  };
}  // providedPorts

BT::NodeStatus SetToPitting::tick()
{
  config().blackboard->set<bool>("is_pitting", getInput<bool>("target_value").value());
  if (getInput<bool>("target_value").value()) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_,
      config()
      .blackboard->get<const std::shared_ptr<ros_parameters::Params>>("rde_params")
      ->non_critical_log_time_period_ms,
      "Pitting, stopping vehicle");
  }
  return BT::NodeStatus::SUCCESS;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::action_nodes
