// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_car_state.hpp"

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

SetCarState::SetCarState(
  const std::string & name, const BT::NodeConfiguration & config,
  const rclcpp::Logger & logger, rclcpp::Clock & clock,
  const race::ttl::TtlTree::ConstSharedPtr ttl_tree)
: BT::SyncActionNode(name, config), logger_(logger), clock_(clock), ttl_tree_(ttl_tree)
{}

BT::PortsList SetCarState::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>(
      "rde_params"),
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs"),
    BT::InputPort<const race::ttl::TtlIndex>("current_ttl_index"),
    BT::InputPort<const race::ttl::TtlIndex>("left_ttl_index"),
    BT::InputPort<const race::ttl::TtlIndex>("right_ttl_index"),
    BT::OutputPort<common::CarState>("previous_car_state"),
    BT::BidirectionalPort<common::CarState>("current_car_state")
  };
}

BT::NodeStatus SetCarState::tick()
{
  bool previous_car_state_exists = true;
  try {
    config().blackboard->set<common::CarState>(
      "previous_car_state",
      config().blackboard->get<common::CarState>("current_car_state"));
  } catch (const std::runtime_error & ex) {
    previous_car_state_exists = false;
  }
  common::CarState car_state;
  if (previous_car_state_exists) {
    car_state = common::ConvertLocalizationStateToCarState(
      config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
        "rde_inputs")->localization_state_msg,
      config().blackboard->get<race::ttl::TtlIndex>("current_ttl_index"),
      config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index"),
      config().blackboard->get<race::ttl::TtlIndex>("right_ttl_index"),
      config().blackboard->get<const common::CarState>("previous_car_state").closest_ttl_index,
      ttl_tree_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->cte_threshold
    );
  } else {
    car_state = common::ConvertLocalizationStateToCarState(
      config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
        "rde_inputs")->localization_state_msg,
      config().blackboard->get<race::ttl::TtlIndex>("current_ttl_index"),
      config().blackboard->get<race::ttl::TtlIndex>("left_ttl_index"),
      config().blackboard->get<race::ttl::TtlIndex>("right_ttl_index"),
      race::ttl::TtlIndex::INVALID,
      ttl_tree_,
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->cte_threshold
    );
  }

  config().blackboard->set<common::CarState>("current_car_state", car_state);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
