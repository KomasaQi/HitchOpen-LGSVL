// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_SEC_RIVAL_CAR_BACK_GAP_LESSER_THAN_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_SEC_RIVAL_CAR_BACK_GAP_LESSER_THAN_HPP_

#include <string>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfSecRivalCarBackGapLesserThan : public BT::ConditionNode
{
public:
  IfSecRivalCarBackGapLesserThan(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};  // class IfSecRivalCarBackGapLesserThan
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_SEC_RIVAL_CAR_BACK_GAP_LESSER_THAN_HPP_
