// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/reset_scale_factor.hpp"

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

ResetScaleFactor::ResetScaleFactor(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList ResetScaleFactor::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::OutputPort<double>("scale_factor")
  };
}

BT::NodeStatus ResetScaleFactor::tick()
{
  config().blackboard->set<double>("scale_factor", common::DEFAULT_SCALE_FACTOR);
  RCLCPP_INFO_THROTTLE(
    logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms, "Resetting scale factor to %f",
    common::DEFAULT_SCALE_FACTOR);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
