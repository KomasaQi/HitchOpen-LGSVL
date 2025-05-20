// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_in_left_ttl.hpp"

#include <memory>
#include <string>

#include "race_decision_engine/common/utils.hpp"
#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "base_common/race_control.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
IfInLeftTtl::IfInLeftTtl(
  const std::string & name, const BT::NodeConfiguration & config,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::ConditionNode(name, config), ttl_tree_(ttl_tree) {}

BT::PortsList IfInLeftTtl::providedPorts()
{
  return {
    BT::InputPort<const race::ttl::TtlIndex>("left_ttl_index"),
    BT::InputPort<const common::CarState>("current_car_state"),
  };
}

BT::NodeStatus IfInLeftTtl::tick()
{
  if (config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index") ==
    config().blackboard->get<const common::CarState>("current_car_state").current_ttl_index)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
