// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/increment_lap.hpp"

#include <string>
#include <set>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/input.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

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

IncrementLap::IncrementLap(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList IncrementLap::providedPorts()
{
  return {
    BT::InputPort<const common::CarState>(
      "previous_car_state"),
    BT::InputPort<const common::CarState>("current_car_state"),
    BT::BidirectionalPort<size_t>("num_laps"),
  };
}

BT::NodeStatus IncrementLap::tick()
{
  config().blackboard->set<size_t>("num_laps", config().blackboard->get<size_t>("num_laps") + 1);
  RCLCPP_INFO(logger_, "Lap Number: %ld", config().blackboard->get<size_t>("num_laps"));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
