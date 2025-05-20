// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_RIVAL_CAR_RIGHT_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_RIVAL_CAR_RIGHT_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
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
class IfRivalCarRight : public BT::ConditionNode
{
public:
  IfRivalCarRight(
    const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_RIVAL_CAR_RIGHT_HPP_
