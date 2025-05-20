// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_ego_car_near_race_line.hpp"

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

IfEgoCarNearRaceLine::IfEgoCarNearRaceLine(
  const std::string & name, const BT::NodeConfiguration & config,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::ConditionNode(name, config), ttl_tree_(ttl_tree)
{}

BT::PortsList IfEgoCarNearRaceLine::providedPorts()
{
  return {
    BT::InputPort<race::ttl::TtlIndex>("race_ttl_index"),
    BT::InputPort<common::CarState>("current_car_state"),
    BT::InputPort<std::shared_ptr<ros_parameters::Params>>("rde_params")
  };
}

BT::NodeStatus IfEgoCarNearRaceLine::tick()
{
  const auto & car_state = config().blackboard->get<common::CarState>("current_car_state");
  const auto & rde_params = config().blackboard->get<std::shared_ptr<ros_parameters::Params>>(
    "rde_params");
  const auto & race_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("race_ttl_index");

  auto race_cte = race::ttl::get_cross_track_error(
    ttl_tree_->get_ttl(race_ttl_index),
    car_state.pos,
    ttl_tree_->find_closest_waypoint_index(race_ttl_index, car_state.pos));

  if (std::abs(race_cte) < rde_params->cte_merge_threshold) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace race::planning::race_decision_engine::bt::condition_nodes
