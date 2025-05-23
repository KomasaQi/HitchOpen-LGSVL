// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/action_nodes/set_push2pass_inactive.hpp"

#include <string>
#include <memory>
#include <set>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace race::planning::race_decision_engine::bt::action_nodes
{

SetPush2PassInactive::SetPush2PassInactive(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetPush2PassInactive::providedPorts()
{
  return {
    BT::BidirectionalPort<bool>("push2pass_cmd")
  };
}

BT::NodeStatus SetPush2PassInactive::tick()
{
  config().blackboard->set<bool>("push2pass_cmd", false);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace race::planning::race_decision_engine::bt::action_nodes
