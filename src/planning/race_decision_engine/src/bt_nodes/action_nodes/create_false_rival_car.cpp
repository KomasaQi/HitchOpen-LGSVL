// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/create_false_rival_car.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/utils.hpp"

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

CreateFalseRivalCar::CreateFalseRivalCar(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList CreateFalseRivalCar::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::OutputPort<common::RivalCar>("rival_car"),
  };
}

BT::NodeStatus CreateFalseRivalCar::tick()
{
  config().blackboard->set<common::RivalCar>(
    "rival_car",
    common::CreateFalseRivalCar());
  RCLCPP_INFO_THROTTLE(
    logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms, "Creating false rival car");
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
