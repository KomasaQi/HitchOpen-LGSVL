// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_FLAG_CHANGED_FROM_WGREEN_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_FLAG_CHANGED_FROM_WGREEN_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
class IfFlagChangedFromWGreen : public BT::ConditionNode
{
public:
  IfFlagChangedFromWGreen(
    const std::string & name, const BT::NodeConfiguration & config,
    const rclcpp::Logger & logger, rclcpp::Clock & clock);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const rclcpp::Logger logger_;
  rclcpp::Clock clock_;
};
}  // namespace race::planning::race_decision_engine::bt::condition_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__CONDITION_NODES__IF_FLAG_CHANGED_FROM_WGREEN_HPP_
