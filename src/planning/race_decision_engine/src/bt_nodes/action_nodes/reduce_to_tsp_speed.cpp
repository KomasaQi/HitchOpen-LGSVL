// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/reduce_to_tsp_speed.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

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

ReduceToTspSpeed::ReduceToTspSpeed(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList ReduceToTspSpeed::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::CarState>("current_car_state"),
    BT::BidirectionalPort<double>("speed_limit"),
    BT::OutputPort<size_t>("tsp_index")
  };
}

BT::NodeStatus ReduceToTspSpeed::tick()
{
  double current_lap_percentage = config().blackboard->get<const common::CarState>(
    "current_car_state").lap_percentage;

  double current_speed_limit = config().blackboard->get<double>("speed_limit");

  auto [upcoming_index, interpolated_speed] = common::InterpolateSpeed(
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->target_speed_profiles.lap_percentages,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->target_speed_profiles.speed_profiles, current_lap_percentage);

  if (interpolated_speed < current_speed_limit) {
    config().blackboard->set<double>("speed_limit", interpolated_speed);
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms,
      "Reducing speed limit to %f at %f", interpolated_speed,
      current_lap_percentage);
  }
  config().blackboard->set<size_t>("tsp_index", upcoming_index);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
