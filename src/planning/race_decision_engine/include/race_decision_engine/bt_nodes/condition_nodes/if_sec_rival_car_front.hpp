// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_SEC_RIVAL_CAR_FRONT_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_SEC_RIVAL_CAR_FRONT_HPP_

#include <string>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfSecRivalCarFront : public BT::ConditionNode
{
public:
  IfSecRivalCarFront(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_SEC_RIVAL_CAR_FRONT_HPP_
