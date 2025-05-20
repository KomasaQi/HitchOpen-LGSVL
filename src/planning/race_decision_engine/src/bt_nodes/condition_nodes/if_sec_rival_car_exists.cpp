// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_exists.hpp"

#include <memory>
#include <string>

#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/utils.hpp"
#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
IfSecRivalCarExists::IfSecRivalCarExists(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}      // IfSecRivalCarExists

BT::PortsList IfSecRivalCarExists::providedPorts()
{
  return {
    BT::InputPort<const common::RivalCar>("current_secondary_rival_car"),
  };
}  // providedPorts

BT::NodeStatus IfSecRivalCarExists::tick()
{
  bool secondary_rival_car_exists =
    config().blackboard->get<const common::RivalCar>("current_secondary_rival_car").exists;
  if (secondary_rival_car_exists) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
