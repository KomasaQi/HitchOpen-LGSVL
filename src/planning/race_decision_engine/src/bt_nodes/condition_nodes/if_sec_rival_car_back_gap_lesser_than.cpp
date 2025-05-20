// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_back_gap_lesser_than.hpp"

#include <memory>
#include <string>

#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/utils.hpp"
#include "race_decision_engine_parameters.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{
IfSecRivalCarBackGapLesserThan::IfSecRivalCarBackGapLesserThan(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config) {}      // IfSecRivalCarBackGapLesserThan

BT::PortsList IfSecRivalCarBackGapLesserThan::providedPorts()
{
  return {BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const common::RivalCar>("current_secondary_rival_car"),
    BT::InputPort<const std::string>("comparison_against")};
}  // providedPorts

BT::NodeStatus IfSecRivalCarBackGapLesserThan::tick()
{
  bool secondary_rival_car_exists =
    config().blackboard->get<const common::RivalCar>("current_secondary_rival_car").exists;
  double secondary_rival_car_back_gap =
    config().blackboard->get<const common::RivalCar>("current_secondary_rival_car").back_gap;
  std::string comparison_against = getInput<std::string>("comparison_against").value();
  if (!secondary_rival_car_exists) {
    return BT::NodeStatus::FAILURE;
  } else {
    std::optional<double> thresh_to_compare_against = common::ReturnComparisonGapThreshold(
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
      comparison_against);
    if (!thresh_to_compare_against) {
      return BT::NodeStatus::FAILURE;
    } else {
      if (secondary_rival_car_back_gap < thresh_to_compare_against.value()) {
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  return BT::NodeStatus::FAILURE;
}  // tick
}  // namespace race::planning::race_decision_engine::bt::condition_nodes
