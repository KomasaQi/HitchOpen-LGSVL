// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_ttl_index.hpp"

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

SetTtlIndex::SetTtlIndex(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetTtlIndex::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<std::string>("selected_ttl"),
    BT::InputPort<race::ttl::TtlIndex>("left_ttl_index"),
    BT::InputPort<race::ttl::TtlIndex>("right_ttl_index"),
    BT::InputPort<race::ttl::TtlIndex>("race_ttl_index"),
    BT::InputPort<race::ttl::TtlIndex>("pit_ttl_index"),
    BT::InputPort<race::ttl::TtlIndex>("optimal_ttl_index"),
    BT::OutputPort<race::ttl::TtlIndex>("output_ttl_index")
  };
}

BT::NodeStatus SetTtlIndex::tick()
{
  auto selected_ttl = config().blackboard->get<std::string>("selected_ttl");
  race::ttl::TtlIndex selected_ttl_index = race::ttl::TtlIndex::INVALID;
  if (selected_ttl == "pit") {
    selected_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("pit_ttl_index");
  } else if (selected_ttl == "right") {
    selected_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("right_ttl_index");
  } else if (selected_ttl == "left") {
    selected_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index");
  } else if (selected_ttl == "race") {
    selected_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("race_ttl_index");
  } else if (selected_ttl == "optimal") {
    selected_ttl_index = config().blackboard->get<race::ttl::TtlIndex>("optimal_ttl_index");
  }
  if (selected_ttl_index != race::ttl::TtlIndex::INVALID) {
    config().blackboard->set<race::ttl::TtlIndex>("output_ttl_index", selected_ttl_index);
  }
  RCLCPP_INFO_THROTTLE(
    logger_, clock_,
    config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms, "Selecting ttl index %ld",
    static_cast<size_t>(config().blackboard->get<race::ttl::TtlIndex>(
      "output_ttl_index")));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
