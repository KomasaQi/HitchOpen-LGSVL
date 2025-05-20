// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_in_passing_zone_from_lookup.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
IfInPassingZoneFromLookup::IfInPassingZoneFromLookup(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList IfInPassingZoneFromLookup::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::CarState>("current_car_state"),
  };
}  // providedPorts

BT::NodeStatus IfInPassingZoneFromLookup::tick()
{
  if (!config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->passing_zones.enabled)
  {
    return BT::NodeStatus::FAILURE;
  }

  if (config().blackboard->get<race::ttl::TtlIndex>(
      "pit_ttl_index") !=
    config().blackboard->get<const common::CarState>(
      "current_car_state").current_ttl_index && ttl_tree_->get_ttl(
      config().blackboard->get<const common::CarState>(
        "current_car_state").current_ttl_index).waypoints.at(
      config().blackboard->get<const common::CarState>(
        "current_car_state").current_ttl_wp_index).region ==
    race::ttl::TrackLocation::STRAIGHT)
  {
    double current_lap_percentage = config().blackboard->get<const common::CarState>(
      "current_car_state").lap_percentage;
    double current_round_speed = config().blackboard->get<double>("current_round_speed");
    double start_percentage = 0.0;
    double end_percentage = 0.0;
    double max_speed = 0.0;
    for (size_t i = 0; i < config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->passing_zones.start_percentages.size(); i++)
    {
      max_speed = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->passing_zones.max_speeds.at(i);
      start_percentage = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->passing_zones.start_percentages.at(i);
      end_percentage = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->passing_zones.end_percentages.at(i);
      if (inside_percentage_range(current_lap_percentage, start_percentage, end_percentage) &&
        current_round_speed <= max_speed)
      {
        RCLCPP_WARN_THROTTLE(
          logger_, clock_,
          config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
            "rde_params")->non_critical_log_time_period_ms, "In Passing Zone");
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  return BT::NodeStatus::FAILURE;
}  // tick

bool IfInPassingZoneFromLookup::inside_percentage_range(
  double current_percentage,
  double start_percentage,
  double end_percentage)
{
  if (start_percentage <= end_percentage) {
    if (current_percentage >= start_percentage &&
      current_percentage <= end_percentage)
    {
      return true;
    }
  } else {
    if (current_percentage >= start_percentage ||
      current_percentage <= end_percentage)
    {
      return true;
    }
  }

  return false;
}  // inside_percentage_range

}  // namespace race::planning::race_decision_engine::bt::condition_nodes
