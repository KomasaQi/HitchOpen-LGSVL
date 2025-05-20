// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_left.hpp"

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

IfRivalCarLeft::IfRivalCarLeft(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IfRivalCarLeft::providedPorts()
{
  return {
    BT::InputPort<const race::ttl::TtlIndex>("left_ttl_index"),
    BT::InputPort<const common::RivalCar>("current_rival_car")
  };
}

BT::NodeStatus IfRivalCarLeft::tick()
{
  bool rival_car_exists =
    config().blackboard->get<const common::RivalCar>("current_rival_car").exists;
  ttl::TtlIndex rival_car_idx =
    config().blackboard->get<const common::RivalCar>("current_rival_car").current_ttl;
  if (!rival_car_exists) {
    return BT::NodeStatus::FAILURE;
  } else {
    if (rival_car_idx == config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index")) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
