// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_ego_car_left.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "ttl.hpp"
#include "ttl_tree.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{

IfEgoCarLeft::IfEgoCarLeft(
  const std::string & name, const BT::NodeConfiguration & config,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::ConditionNode(name, config), ttl_tree_(ttl_tree)
{}

BT::PortsList IfEgoCarLeft::providedPorts()
{
  return {
    BT::InputPort<race::ttl::TtlIndex>("left_ttl_index"),
    BT::InputPort<race::ttl::TtlIndex>("right_ttl_index"),
    BT::InputPort<common::CarState>("current_car_state"),
    BT::InputPort<std::shared_ptr<ros_parameters::Params>>("rde_params")
  };
}

BT::NodeStatus IfEgoCarLeft::tick()
{
  const auto & car_state = config().blackboard->get<common::CarState>("current_car_state");
  const auto & rde_params = config().blackboard->get<std::shared_ptr<ros_parameters::Params>>(
    "rde_params");
  const auto & left_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index");
  const auto & right_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("right_ttl_index");

  race::ttl::TtlIndex half_index = common::GetLineNominal(
    car_state.pos,
    ttl_tree_,
    rde_params->cte_threshold,
    left_ttl_index,
    right_ttl_index,
    car_state.current_ttl_index
  );

  if (left_ttl_index == half_index) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace race::planning::race_decision_engine::bt::condition_nodes
