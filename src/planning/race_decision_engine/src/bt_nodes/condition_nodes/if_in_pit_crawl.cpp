// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_crawl.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "base_common/race_control.hpp"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
IfInPitCrawl::IfInPitCrawl(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList IfInPitCrawl::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const race::ttl::TtlIndex>(
      "pit_ttl_index"),
    BT::InputPort<const common::CarState>(
      "current_car_state"),
  };
}

BT::NodeStatus IfInPitCrawl::tick()
{
  if (config().blackboard->get<race::ttl::TtlIndex>(
      "pit_ttl_index") ==
    config().blackboard->get<const common::CarState>(
      "current_car_state").current_ttl_index && ttl_tree_->get_ttl(
      config().blackboard->get<const common::CarState>(
        "current_car_state").current_ttl_index).waypoints.at(
      config().blackboard->get<const common::CarState>(
        "current_car_state").current_ttl_wp_index).region ==
    race::ttl::TrackLocation::PIT_CRAWL)
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "In Pit Crawl");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
