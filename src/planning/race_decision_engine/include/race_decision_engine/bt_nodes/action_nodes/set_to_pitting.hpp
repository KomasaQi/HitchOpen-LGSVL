// Copyright 2024 Moises Lopez

#ifndef RACE_DECISION_ENGINE__BT_NODES__ACTION_NODES__SET_TO_PITTING_HPP_
#define RACE_DECISION_ENGINE__BT_NODES__ACTION_NODES__SET_TO_PITTING_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

namespace race::planning::race_decision_engine::bt::action_nodes
{
class SetToPitting : public BT::SyncActionNode
{
public:
  SetToPitting(
    const std::string & name, const BT::NodeConfiguration & config,
    const rclcpp::Logger & logger, rclcpp::Clock & clock);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  const rclcpp::Logger logger_;
  rclcpp::Clock clock_;
};  // class SetToPitting
}  // namespace race::planning::race_decision_engine::bt::action_nodes

#endif  // RACE_DECISION_ENGINE__BT_NODES__ACTION_NODES__SET_TO_PITTING_HPP_
