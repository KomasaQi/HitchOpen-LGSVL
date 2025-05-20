// Copyright 2023 Siddharth Saha

#ifndef RACE_DECISION_ENGINE__BT_NODES__ACTION_NODES__REDUCE_TO_TSP_SPEED_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__ACTION_NODES__REDUCE_TO_TSP_SPEED_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "ttl_tree.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace bt
{
namespace action_nodes
{
class ReduceToTspSpeed : public BT::SyncActionNode
{
public:
  ReduceToTspSpeed(
    const std::string & name, const BT::NodeConfiguration & config,
    const rclcpp::Logger & logger, rclcpp::Clock & clock,
    const race::ttl::TtlTree::ConstSharedPtr ttl_tree);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree_;
};
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

#endif  // RACE_DECISION_ENGINE__BT_NODES__ACTION_NODES__REDUCE_TO_TSP_SPEED_HPP_
