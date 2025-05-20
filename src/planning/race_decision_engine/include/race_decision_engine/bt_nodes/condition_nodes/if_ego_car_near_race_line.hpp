// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_EGO_CAR_NEAR_RACE_LINE_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_EGO_CAR_NEAR_RACE_LINE_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "ttl_tree.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfEgoCarNearRaceLine : public BT::ConditionNode
{
public:
  IfEgoCarNearRaceLine(
    const std::string & name, const BT::NodeConfiguration & config,
    const race::ttl::TtlTree::ConstSharedPtr ttl_tree);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree_;
};
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_EGO_CAR_NEAR_RACE_LINE_HPP_
