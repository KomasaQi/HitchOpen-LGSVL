// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/action_nodes/set_allowed_to_overtake.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "race_msgs/msg/stop_type.hpp"
#include "race_msgs/msg/strategy_type.hpp"

namespace race::planning::race_decision_engine::bt::action_nodes
{
SetAllowedToOvertake::SetAllowedToOvertake(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetAllowedToOvertake::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<bool>("target_value"),
    BT::OutputPort<bool>("allowed_to_overtake"),
  };
}

BT::NodeStatus SetAllowedToOvertake::tick()
{
  config().blackboard->set<bool>(
    "allowed_to_overtake",
    getInput<bool>("target_value").value());

  if (getInput<bool>("target_value").value()) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_,
      config()
      .blackboard->get<const std::shared_ptr<ros_parameters::Params>>("rde_params")
      ->non_critical_log_time_period_ms,
      "Setting allowed_to_overtake to %d",
      getInput<bool>("target_value").value());
  }

  return BT::NodeStatus::SUCCESS;
}
}  // namespace race::planning::race_decision_engine::bt::action_nodes
