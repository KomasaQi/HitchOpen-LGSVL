// Copyright 2023 Siddharth Saha

#include "race_decision_engine/impl/iac_bt.hpp"

#include <string>
#include <set>

#include "race_decision_engine/common/utils.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace nodes
{
namespace implementations
{
IacBt::IacBt(const rclcpp::NodeOptions & options)
: base::RaceDecisionEngineBtBaseLifecycleNode(options)
{}

void IacBt::Initialize()
{
  RaceDecisionEngineBtBaseLifecycleNode::Initialize();
  GetBlackboard()->set<size_t>("num_laps", GetParams()->num_start_laps);
  GetBlackboard()->set<double>("actual_lap_distance", 0);
  if (GetParams()->start_in_pit) {
    GetBlackboard()->set<const race::ttl::TtlIndex>("current_ttl_index", GetPitTtlIndex());
    GetBlackboard()->set<std::string>("selected_ttl", "pit");
  } else {
    GetBlackboard()->set<const race::ttl::TtlIndex>("current_ttl_index", GetRaceTtlIndex());
    GetBlackboard()->set<std::string>("selected_ttl", "race");
  }
  GetBlackboard()->set<std::string>("speed_type", "stop");
  GetBlackboard()->set<std::string>("gap_type", "no_gap");
  GetBlackboard()->set<uint8_t>("strategy_type", race_msgs::msg::StrategyType::CRUISE_CONTROL);
  GetBlackboard()->set<common::RivalCar>("current_rival_car", common::CreateFalseRivalCar());
  GetBlackboard()->set<size_t>("tsp_index", 0);
  GetBlackboard()->set<bool>("allowed_to_overtake", false);
  GetBlackboard()->set<bool>("defender_overtake_complete", false);
  GetBlackboard()->set<bool>("attacker_overtake_complete", false);
  GetBlackboard()->set<bool>("defender_overtake_started", false);
  GetBlackboard()->set<bool>("attacker_overtake_started", false);
}

void IacBt::RunTree()
{
  GetBlackboard()->set<const race::ttl::TtlIndex>("left_ttl_index", GetLeftTtlIndex());
  GetBlackboard()->set<const race::ttl::TtlIndex>("right_ttl_index", GetRightTtlIndex());
  GetBlackboard()->set<const race::ttl::TtlIndex>("race_ttl_index", GetRaceTtlIndex());
  GetBlackboard()->set<const race::ttl::TtlIndex>("pit_ttl_index", GetPitTtlIndex());
  GetBlackboard()->set<const race::ttl::TtlIndex>("optimal_ttl_index", GetOptimalTtlIndex());
  GetBtTree().tickRoot();
}

void IacBt::PostRunTree()
{
  GetBlackboard()->set<race::ttl::TtlIndex>(
    "current_ttl_index",
    GetBlackboard()->get<race::ttl::TtlIndex>("output_ttl_index"));
}
}  // namespace implementations
}  // namespace nodes
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race

RCLCPP_COMPONENTS_REGISTER_NODE(
  race::planning::race_decision_engine::nodes::implementations::IacBt)
