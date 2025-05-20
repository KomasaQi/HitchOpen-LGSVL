// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/action_nodes/create_false_sec_rival_car.hpp"

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

namespace race::planning::race_decision_engine::bt::action_nodes
{

CreateFalseSecRivalCar::CreateFalseSecRivalCar(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList CreateFalseSecRivalCar::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::OutputPort<common::RivalCar>("sec_rival_car"),
  };
}

BT::NodeStatus CreateFalseSecRivalCar::tick()
{
  config().blackboard->set<common::RivalCar>(
    "sec_rival_car",
    common::CreateFalseRivalCar());
  RCLCPP_INFO_THROTTLE(
    logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params")->non_critical_log_time_period_ms, "Creating secondary false rival car");
  return BT::NodeStatus::SUCCESS;
}
}  // namespace race::planning::race_decision_engine::bt::action_nodes
