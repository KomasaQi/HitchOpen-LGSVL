// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_in_yellow_speed_range.hpp"

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
IfInYellowSpeedRange::IfInYellowSpeedRange(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}      // IfInYellowSpeedRange

BT::PortsList IfInYellowSpeedRange::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const common::CarState>("current_car_state"),
  };
}  // providedPorts

BT::NodeStatus IfInYellowSpeedRange::tick()
{
  double yellow_speed =
    config()
    .blackboard->get<const std::shared_ptr<ros_parameters::Params>>("rde_params")
    ->speed_limit.yellow;

  auto current_speed = config().blackboard->get<const common::CarState>("current_car_state").speed;

  if ((current_speed / common::MPH_TO_MPS) <= (yellow_speed + 5.0)) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
