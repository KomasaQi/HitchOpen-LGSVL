// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_gap.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "race_msgs/msg/stop_type.hpp"
#include "race_msgs/msg/strategy_type.hpp"
#include "ttl.hpp"

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

SetGap::SetGap(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetGap::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<std::string>("gap_type"),
    BT::OutputPort<double>("target_gap"),
  };
}

BT::NodeStatus SetGap::tick()
{
  auto gap_type = config().blackboard->get<std::string>("gap_type");
  double target_gap = 0.0;
  if (gap_type == "no_gap") {
    target_gap = 0.0;
  } else if (gap_type == "attacker_preparing_gap") {
    target_gap = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->gaps.attacker_preparing;
  } else if (gap_type == "attacker_attacking_gap") {
    target_gap = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->gaps.attacker_attacking;
  } else if (gap_type == "defender_overtaken_gap") {
    target_gap = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->gaps.defender_overtaken;
  }
  config().blackboard->set<double>("target_gap", target_gap);
  RCLCPP_INFO_THROTTLE(
    logger_, clock_,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms,
    "Setting gap %f based on gap type %s",
    config().blackboard->get<double>("target_gap"), gap_type.c_str());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
