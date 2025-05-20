// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_highest_speed_rival_car.hpp"

#include <string>
#include <memory>
#include <set>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "race_decision_engine/common/utils.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

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

SetHighestSpeedRivalCar::SetHighestSpeedRivalCar(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock)
{}

BT::PortsList SetHighestSpeedRivalCar::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs"),
    BT::OutputPort<common::RivalCar>("rival_car"),
  };
}

BT::NodeStatus SetHighestSpeedRivalCar::tick()
{
  const auto opp_detections_msg = config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
    "rde_inputs")->opp_car_detections_msg;
  if (opp_detections_msg->objects.size() == 0) {
    config().blackboard->set<common::RivalCar>(
      "rival_car",
      common::CreateFalseRivalCar());
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "Creating false rival car");
  } else {
    auto highest_speed_rival_car = common::GetHighestSpeedRivalCar(opp_detections_msg);
    config().blackboard->set<common::RivalCar>(
      "rival_car", highest_speed_rival_car);
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms,
      "Setting highest speed rival car with speed = %f",
      highest_speed_rival_car.speed);
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
