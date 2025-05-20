// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_pitting.hpp"

#include <memory>
#include <set>
#include <string>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
IfPitting::IfPitting(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}      // IfPitting

BT::PortsList IfPitting::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::OutputPort<bool>("is_pitting"),
  };
}  // providedPorts

BT::NodeStatus IfPitting::tick()
{
  if (config().blackboard->get<bool>("is_pitting")) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
