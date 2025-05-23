// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/action_nodes/set_second_closest_rival_car.hpp"

#include <memory>
#include <set>
#include <string>

#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "race_decision_engine/common/utils.hpp"
#include "ttl_tree.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace race::planning::race_decision_engine::bt::action_nodes
{
SetSecondClosestRivalCar::SetSecondClosestRivalCar(
  const std::string & name, const BT::NodeConfiguration & config, const rclcpp::Logger & logger,
  rclcpp::Clock & clock, const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{
}  // SetSecondClosestRivalCar

BT::PortsList SetSecondClosestRivalCar::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>("rde_inputs"),
    BT::InputPort<const common::CarState>("current_car_state"),
    BT::OutputPort<common::RivalCar>("sec_rival_car"),
  };
}  // providedPorts

BT::NodeStatus SetSecondClosestRivalCar::tick()
{
  const auto opp_detections_msg =
    config()
    .blackboard->get<const common::RdeInputs::ConstSharedPtr>("rde_inputs")
    ->opp_car_detections_msg;

  // If there are no detections or only one detection, create a false secondary rival car
  if (opp_detections_msg->objects.size() <= 1) {
    config().blackboard->set<common::RivalCar>(
      "sec_rival_car",
      common::CreateFalseRivalCar());
    RCLCPP_INFO_THROTTLE(
      logger_, clock_,
      config()
      .blackboard->get<const std::shared_ptr<ros_parameters::Params>>("rde_params")
      ->non_critical_log_time_period_ms,
      "Creating false secondary rival car");
    return BT::NodeStatus::SUCCESS;
  }

  // Set the second closest rival car
  auto rival_car = common::GetSecondClosestRivalCar(
    opp_detections_msg, config().blackboard->get<const common::CarState>("current_car_state"),
    ttl_tree_);

  config().blackboard->set<common::RivalCar>("sec_rival_car", rival_car);

  RCLCPP_INFO_THROTTLE(
    logger_, clock_,
    config()
    .blackboard->get<const std::shared_ptr<ros_parameters::Params>>("rde_params")
    ->non_critical_log_time_period_ms,
    "Setting second closest rival car with speed = %f", rival_car.speed);

  return BT::NodeStatus::SUCCESS;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::action_nodes
