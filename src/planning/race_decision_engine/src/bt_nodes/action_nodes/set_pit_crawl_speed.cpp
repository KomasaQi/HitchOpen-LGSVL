// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/action_nodes/set_pit_crawl_speed.hpp"

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

namespace race::planning::race_decision_engine::bt::action_nodes
{

SetPitCrawlSpeed::SetPitCrawlSpeed(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetPitCrawlSpeed::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::OutputPort<std::string>("speed_type"),
    BT::OutputPort<uint8_t>("stop_type"),
    BT::OutputPort<uint8_t>("strategy_type")
  };
}

BT::NodeStatus SetPitCrawlSpeed::tick()
{
  config().blackboard->set<std::string>("speed_type", "pit_crawl");
  config().blackboard->set<uint8_t>("stop_type", race_msgs::msg::StopType::STOP_TYPE_NOMINAL);
  config().blackboard->set<uint8_t>("strategy_type", race_msgs::msg::StrategyType::CRUISE_CONTROL);
  RCLCPP_INFO_THROTTLE(
    logger_, clock_,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms, "Pit crawl speed");
  return BT::NodeStatus::SUCCESS;
}
}  // namespace race::planning::race_decision_engine::bt::action_nodes
