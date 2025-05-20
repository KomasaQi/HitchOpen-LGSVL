// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_exit.hpp"

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

IfInPitExit::IfInPitExit(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList IfInPitExit::providedPorts()
{
  return {
    BT::InputPort<const race::ttl::TtlIndex>(
      "pit_ttl_index"),
    BT::InputPort<const common::CarState>(
      "current_car_state"),
  };
}

BT::NodeStatus IfInPitExit::tick()
{
  if (config().blackboard->get<race::ttl::TtlIndex>(
      "pit_ttl_index") ==
    config().blackboard->get<const common::CarState>(
      "current_car_state").current_ttl_index && ttl_tree_->get_ttl(
      config().blackboard->get<const common::CarState>(
        "current_car_state").current_ttl_index).waypoints.at(
      config().blackboard->get<const common::CarState>(
        "current_car_state").current_ttl_wp_index).region ==
    race::ttl::TrackLocation::PIT_EXIT)
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "In Pit Exit");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
