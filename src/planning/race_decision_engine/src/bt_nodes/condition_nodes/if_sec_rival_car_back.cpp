// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_back.hpp"

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
IfSecRivalCarBack::IfSecRivalCarBack(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}

BT::PortsList IfSecRivalCarBack::providedPorts()
{
  return {BT::InputPort<const common::RivalCar>("current_secondary_rival_car")};
}

BT::NodeStatus IfSecRivalCarBack::tick()
{
  bool secondary_rival_car_exists =
    config().blackboard->get<const common::RivalCar>("current_secondary_rival_car").exists;
  double secondary_rival_car_front_gap =
    config().blackboard->get<const common::RivalCar>("current_secondary_rival_car").front_gap;
  double secondary_rival_car_back_gap =
    config().blackboard->get<const common::RivalCar>("current_secondary_rival_car").back_gap;
  if (!secondary_rival_car_exists) {
    return BT::NodeStatus::FAILURE;
  } else {
    if (std::abs(secondary_rival_car_front_gap) > std::abs(secondary_rival_car_back_gap)) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
