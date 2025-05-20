// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_scale_factor_from_lookup.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

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

SetScaleFactorFromLookup::SetScaleFactorFromLookup(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList SetScaleFactorFromLookup::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::CarState>("current_car_state"),
    BT::OutputPort<double>("scale_factor"),
    BT::OutputPort<size_t>("sf_index")
  };
}

BT::NodeStatus SetScaleFactorFromLookup::tick()
{
  double current_lap_percentage = config().blackboard->get<const common::CarState>(
    "current_car_state").lap_percentage;

  size_t sf_index = 0;
  for (size_t i = 1; i < config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->scale_factors.lap_percentages.size(); i++)
  {
    if (config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->scale_factors.lap_percentages.at(i) > current_lap_percentage)
    {
      config().blackboard->set<double>(
        "scale_factor", config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
          "rde_params")->scale_factors.scale_factors.at(i - 1));
      sf_index = i - 1;
      RCLCPP_INFO_THROTTLE(
        logger_, clock_,
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
          "rde_params")->non_critical_log_time_period_ms,
        "Setting scale factor to %f at %f",
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
          "rde_params")->scale_factors.scale_factors.at(i - 1), current_lap_percentage);
      break;
    }
  }

  if (current_lap_percentage >= common::MAX_LAP_PERCENTAGE) {
    config().blackboard->set<double>(
      "scale_factor", config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->scale_factors.scale_factors.back());
    sf_index = config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->scale_factors.scale_factors.size() - 1;
  }

  config().blackboard->set<size_t>("sf_index", sf_index);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
