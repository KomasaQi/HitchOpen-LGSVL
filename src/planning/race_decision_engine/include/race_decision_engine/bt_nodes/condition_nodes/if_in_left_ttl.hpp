// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_LEFT_TTL_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_LEFT_TTL_HPP_

#include <string>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "ttl_tree.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfInLeftTtl : public BT::ConditionNode
{
public:
  IfInLeftTtl(
    const std::string & name, const BT::NodeConfiguration & config,
    const race::ttl::TtlTree::ConstSharedPtr ttl_tree);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree_;
};  // class IfInLeftTtl
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_LEFT_TTL_HPP_
