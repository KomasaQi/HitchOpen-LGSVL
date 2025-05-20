// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_in_round_speed_range.hpp"

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
IfInRoundSpeedRange::IfInRoundSpeedRange(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}      // IfInRoundSpeedRange

BT::PortsList IfInRoundSpeedRange::providedPorts()
{
  return {
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>("rde_inputs"),
    BT::InputPort<const common::CarState>("current_car_state"),
  };
}  // providedPorts

BT::NodeStatus IfInRoundSpeedRange::tick()
{
  double round_speed = config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
    "rde_inputs")->race_control_msg->round_target_speed;

  auto current_speed = config().blackboard->get<const common::CarState>("current_car_state").speed;

  if ((current_speed / common::MPH_TO_MPS) <= (round_speed + 5.0)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
