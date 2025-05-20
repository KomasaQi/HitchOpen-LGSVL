// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_back_gap_greater_than.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/input.hpp"
#include "race_decision_engine/common/utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace bt
{
namespace condition_nodes
{

IfRivalCarBackGapGreaterThan::IfRivalCarBackGapGreaterThan(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IfRivalCarBackGapGreaterThan::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const common::RivalCar>("current_rival_car"),
    BT::InputPort<const std::string>("comparison_against")
  };
}

BT::NodeStatus IfRivalCarBackGapGreaterThan::tick()
{
  bool rival_car_exists =
    config().blackboard->get<const common::RivalCar>("current_rival_car").exists;
  double rival_car_back_gap =
    config().blackboard->get<const common::RivalCar>("current_rival_car").back_gap;
  std::string comparison_against = getInput<std::string>("comparison_against").value();
  if (!rival_car_exists) {
    return BT::NodeStatus::FAILURE;
  } else {
    std::optional<double> thresh_to_compare_against = common::ReturnComparisonGapThreshold(
      config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params"), comparison_against);
    if (!thresh_to_compare_against) {
      return BT::NodeStatus::FAILURE;
    } else {
      if (rival_car_back_gap > thresh_to_compare_against.value()) {
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
  return BT::NodeStatus::FAILURE;
}
}  // namespace condition_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
