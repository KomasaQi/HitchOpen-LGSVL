// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/fill_rival_car_info.hpp"

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
#include "ttl.hpp"
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

FillRivalCarInfo::FillRivalCarInfo(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList FillRivalCarInfo::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs"),
    BT::InputPort<const race::ttl::TtlIndex>("current_ttl_index"),
    BT::InputPort<const race::ttl::TtlIndex>("left_ttl_index"),
    BT::InputPort<const race::ttl::TtlIndex>("right_ttl_index"),
    BT::InputPort<const common::RivalCar>("rival_car"),
    BT::OutputPort<common::RivalCar>("previous_rival_car"),
    BT::BidirectionalPort<common::RivalCar>("current_rival_car")
  };
}

BT::NodeStatus FillRivalCarInfo::tick()
{
  bool previous_rival_car_exists = true;
  common::RivalCar previous_rival_car;
  try {
    previous_rival_car = config().blackboard->get<common::RivalCar>("current_rival_car");
    config().blackboard->set<common::RivalCar>(
      "previous_rival_car", previous_rival_car);
  } catch (const std::runtime_error & ex) {
    previous_rival_car_exists = false;
    previous_rival_car = common::CreateFalseRivalCar();
  }
  common::RivalCar rival_car = config().blackboard->get<common::RivalCar>("rival_car");
  if (previous_rival_car_exists) {
    rival_car.current_ttl = previous_rival_car.current_ttl;
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms, "Rival car latched onto line %d",
      static_cast<uint8_t>(rival_car.current_ttl));
  } else {
    rival_car.current_ttl = race::ttl::TtlIndex::INVALID;
  }
  if (rival_car.exists) {
    common::FindRivalCarHalf(
      rival_car,
      ttl_tree_,
      config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index"),
      config().blackboard->get<race::ttl::TtlIndex>("right_ttl_index"),
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->cte_threshold
    );
    common::FillRivalCarGaps(
      config().blackboard->get<const common::CarState>("current_car_state"),
      rival_car,
      ttl_tree_
    );
    RCLCPP_INFO_THROTTLE(
      logger_, clock_, config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->non_critical_log_time_period_ms,
      "Rival car tracked on line %d with front gap %f and back gap %f",
      static_cast<int>(rival_car.current_ttl), rival_car.front_gap, rival_car.back_gap);
  }

  config().blackboard->set<common::RivalCar>("current_rival_car", rival_car);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
