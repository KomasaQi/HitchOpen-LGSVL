// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_green_speed_flag.hpp"

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
IfGreenSpeedFlag::IfGreenSpeedFlag(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::ConditionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList IfGreenSpeedFlag::providedPorts()
{
  return {
    BT::InputPort<const race::Flags>(
      "current_flags")
  };
}

BT::NodeStatus IfGreenSpeedFlag::tick()
{
  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g40_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G40 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g60_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G60 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g80_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G80 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g100_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G100 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g120_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G120 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g130_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G130 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g140_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G140 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g145_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G145 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g150_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G150 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g155_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G155 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g160_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G160 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g165_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G165 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g170_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G170 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g175_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G175 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g180_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G180 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g185_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G185 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  if (config().blackboard->get<const race::Flags>(
      "current_flags").has_g190_flag())
  {
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "G190 Flag");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
