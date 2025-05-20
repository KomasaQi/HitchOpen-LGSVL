// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/print_valid_ttl.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
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

PrintValidTtl::PrintValidTtl(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList PrintValidTtl::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")
  };
}

BT::NodeStatus PrintValidTtl::tick()
{
  std::string valid_idx = "";
  for (const auto & index : ttl_tree_->get_valid_ttl_index_set()) {
    valid_idx += (std::to_string(static_cast<unsigned>(index)) + " ");
  }
  RCLCPP_INFO_THROTTLE(
    logger_, clock_,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms, valid_idx.c_str());
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
