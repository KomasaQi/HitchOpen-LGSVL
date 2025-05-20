// Copyright 2024 Moises Lopez

#include "race_decision_engine/bt_nodes/condition_nodes/if_crossed_passing_line.hpp"

#include <string>
#include <memory>

#include "race_decision_engine_parameters.hpp"
#include "race_decision_engine/common/input.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

namespace race::planning::race_decision_engine::bt::condition_nodes
{

IfCrossedPassingLine::IfCrossedPassingLine(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{}

BT::PortsList IfCrossedPassingLine::providedPorts()
{
  return {
    BT::InputPort<const std::shared_ptr<ros_parameters::Params>>("rde_params"),
    BT::InputPort<const common::CarState>("previous_car_state"),
    BT::InputPort<const common::CarState>("current_car_state")
  };
}

BT::NodeStatus IfCrossedPassingLine::tick()
{
  try {
    double current_lap_percentage = config().blackboard->get<const common::CarState>(
      "current_car_state").lap_percentage;
    double previous_lap_percentage = config().blackboard->get<const common::CarState>(
      "previous_car_state").lap_percentage;
    for (size_t i = 0; i < config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->passing_lines.percentages.size(); i++)
    {
      auto percent_to_check =
        config().blackboard->get<const std::shared_ptr<ros_parameters::Params>>(
        "rde_params")->passing_lines.percentages.at(i);
      auto check_sf = current_lap_percentage < previous_lap_percentage &&
        previous_lap_percentage >= 95.0 && current_lap_percentage < 5.0;
      if (previous_lap_percentage < percent_to_check &&
        current_lap_percentage >= percent_to_check)
      {
        return BT::NodeStatus::SUCCESS;
      } else if (check_sf && (percent_to_check >= 95.0 || percent_to_check < 5.0)) {
        return BT::NodeStatus::SUCCESS;
      }
    }
  } catch (const std::runtime_error & ex) {
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace race::planning::race_decision_engine::bt::condition_nodes
