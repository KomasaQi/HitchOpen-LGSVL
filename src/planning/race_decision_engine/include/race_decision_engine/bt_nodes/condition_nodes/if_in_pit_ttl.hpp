// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PIT_TTL_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PIT_TTL_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "ttl_tree.hpp"

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
class IfInPitTtl : public BT::ConditionNode
{
public:
  IfInPitTtl(
    const std::string & name, const BT::NodeConfiguration & config,
    const race::ttl::TtlTree::ConstSharedPtr ttl_tree);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree_;
};
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PIT_TTL_HPP_
