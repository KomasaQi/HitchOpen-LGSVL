// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PASSING_ZONE_FROM_LOOKUP_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PASSING_ZONE_FROM_LOOKUP_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "ttl_tree.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfInPassingZoneFromLookup : public BT::ConditionNode
{
public:
  IfInPassingZoneFromLookup(
    const std::string & name, const BT::NodeConfiguration & config,
    const rclcpp::Logger & logger, rclcpp::Clock & clock,
    const race::ttl::TtlTree::ConstSharedPtr ttl_tree);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree_;

  bool inside_percentage_range(
    double current_percentage,
    double start_percentage,
    double end_percentage);
};
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_IN_PASSING_ZONE_FROM_LOOKUP_HPP_
