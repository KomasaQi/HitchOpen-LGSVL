// Copyright 2023 Siddharth Saha

#include "race_decision_engine/bt_nodes/action_nodes/set_flags.hpp"

#include <string>
#include <set>

#include "race_decision_engine/common/input.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "base_common/race_control.hpp"

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

SetFlags::SetFlags(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList SetFlags::providedPorts()
{
  return {
    BT::InputPort<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs"),
    BT::OutputPort<race::Flags>("previous_flags"),
    BT::OutputPort<double>("current_round_speed"),
    BT::BidirectionalPort<race::Flags>("current_flags")
  };
}

BT::NodeStatus SetFlags::tick()
{
  try {
    config().blackboard->set<race::Flags>(
      "previous_flags",
      config().blackboard->get<race::Flags>("current_flags"));
  } catch (const std::runtime_error & ex) {
    config().blackboard->set<race::Flags>(
      "previous_flags",
      Flags(static_cast<FlagsBitfield>(0))
    );
  }
  race::Flags flags =
    get_flags(
    *config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")->race_control_msg);
  config().blackboard->set<race::Flags>("current_flags", flags);
  config().blackboard->set<double>(
    "current_round_speed", config().blackboard->get<const common::RdeInputs::ConstSharedPtr>(
      "rde_inputs")->race_control_msg->round_target_speed);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace action_nodes
}  // namespace bt
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
