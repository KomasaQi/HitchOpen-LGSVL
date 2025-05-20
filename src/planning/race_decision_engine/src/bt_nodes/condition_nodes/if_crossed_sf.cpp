// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_crossed_sf.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/input.hpp"

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

IfCrossedSf::IfCrossedSf(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IfCrossedSf::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const common::CarState>("previous_car_state"),
    BT::InputPort<const common::CarState>("current_car_state")
  };
}

BT::NodeStatus IfCrossedSf::tick()
{
  try {
    if (config().blackboard->get<const common::CarState>("previous_car_state").lap_distance <
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->lap_threshold &&
      config().blackboard->get<const common::CarState>("current_car_state").lap_distance >=
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->lap_threshold)
    {
      return BT::NodeStatus::SUCCESS;
    }
  } catch (const std::runtime_error & ex) {
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
