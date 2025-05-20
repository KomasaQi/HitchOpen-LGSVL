// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_ego_ttls_match.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

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

IfRivalEgoTtlsMatch::IfRivalEgoTtlsMatch(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IfRivalEgoTtlsMatch::providedPorts()
{
  return {
    BT::InputPort<const common::RivalCar>("current_rival_car"),
    BT::InputPort<const common::CarState>("current_car_state")
  };
}

BT::NodeStatus IfRivalEgoTtlsMatch::tick()
{
  auto rival_car = config().blackboard->get<const common::RivalCar>("current_rival_car");
  if (!rival_car.exists) {
    return BT::NodeStatus::FAILURE;
  }
  if (rival_car.current_ttl ==
    config().blackboard->get<const common::CarState>("current_car_state").closest_ttl_index)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
