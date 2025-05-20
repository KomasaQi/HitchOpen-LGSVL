// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_YELLOW_SPEED_RANGE_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_YELLOW_SPEED_RANGE_HPP_

#include <string>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfInYellowSpeedRange : public BT::ConditionNode
{
public:
  IfInYellowSpeedRange(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_YELLOW_SPEED_RANGE_HPP_
