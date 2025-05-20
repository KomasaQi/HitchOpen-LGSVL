// Copyright 2023 Siddharth Saha

#include "race_decision_engine/base/base_bt_lifecycle_node.hpp"

#include <memory>
#include <set>
#include <string>

#include "race_decision_engine/bt_nodes/action_nodes/create_false_rival_car.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/create_false_sec_rival_car.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/fill_rival_car_info.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/fill_sec_rival_car_info.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/go_joystick_autonomous.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/increment_lap.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/keep_joystick_state.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/print_valid_ttl.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/reduce_to_tsp_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/reset_scale_factor.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_allowed_to_overtake.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_attacker_attacking_gap.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_attacker_overtake_complete.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_attacker_overtake_started.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_attacker_preparing_gap.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_car_state.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_closest_rival_car.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_defender_overtake_complete.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_defender_overtake_started.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_defender_overtaken_gap.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_flags.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_follow_mode.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_gap.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_green_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_highest_speed_rival_car.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_joystick_state.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_left_ttl_index.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_no_gap.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_optimal_ttl_index.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_pit_crawl_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_pit_lane_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_pit_road_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_pit_ttl_index.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_push2pass_active.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_push2pass_inactive.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_race_ttl_index.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_right_ttl_index.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_round_catchup_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_round_overtaking_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_round_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_scale_factor_from_lookup.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_second_closest_rival_car.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_speed_limit.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_to_pitting.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_ttl_index.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_yellow_catchup_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/set_yellow_speed.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/stop_car_emergency.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/stop_car_immediate.hpp"
#include "race_decision_engine/bt_nodes/action_nodes/stop_car_safe.hpp"

#include "race_decision_engine/bt_nodes/condition_nodes/if_allowed_to_overtake.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_attacker_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_attacker_overtake_complete.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_attacker_overtake_started.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_black_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_blue_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_checkered_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_crossed_passing_sf.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_crossed_passing_line.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_crossed_sf.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_defender_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_defender_overtake_complete.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_defender_overtake_started.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_ego_car_left.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_ego_car_near_race_line.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_ego_car_right.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_ekill_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_fcy_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_flag_changed_from_green.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_flag_changed_from_wgreen.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_green_flag.hpp"
#include  "race_decision_engine/bt_nodes/condition_nodes/if_green_speed_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_back_straight.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_left_ttl.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_passing_zone_from_lookup.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_box.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_crawl.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_entrance.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_exit.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_lane.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_road.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_pit_ttl.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_round_speed_range.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_straight.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_in_yellow_speed_range.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_input_manual_command_emergency.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_input_manual_command_timeout.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_localization_emergency.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_localization_timeout.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_low_level_fault_emergency_stop.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_low_level_fault_immediate_stop.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_low_level_fault_safe_stop.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_low_level_fault_timeout.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_no_track_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_no_tracked_objects.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_opp_car_detection_timeout.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_orange_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_pitting.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_purple_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_race_control_timeout.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_red_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_back.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_back_gap_greater_than.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_back_gap_lesser_than.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_exists.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_front.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_front_gap_greater_than.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_front_gap_lesser_than.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_left.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_car_right.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_rival_ego_ttls_match.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_scale_factor_enabled.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_back_gap_lesser_than.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_front_gap_lesser_than.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_back.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_exists.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_sec_rival_car_front.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_stop_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_tsp_enabled.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_use_params.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_use_perception.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_wgreen_flag.hpp"
#include "race_decision_engine/bt_nodes/condition_nodes/if_yellow_flag.hpp"

namespace race
{
namespace planning
{
namespace race_decision_engine
{
namespace nodes
{
namespace base
{
RaceDecisionEngineBtBaseLifecycleNode::RaceDecisionEngineBtBaseLifecycleNode(
  const rclcpp::NodeOptions & options)
: RaceDecisionEngineBaseLifecycleNode(options)
{
  RCLCPP_INFO(this->get_logger(), "Created BT Base Node");
}

void RaceDecisionEngineBtBaseLifecycleNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initialized BT Base Node");
  blackboard_ = BT::Blackboard::create();
  RegisterTreeNodes();
  bt_tree_ = factory_.createTreeFromFile(GetParams()->behavior_tree.bt_fp, blackboard_);
  groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
    bt_tree_, static_cast<unsigned>(GetParams()->behavior_tree.zmq_max_msgs_per_sec),
    static_cast<unsigned>(GetParams()->behavior_tree.groot_publisher_port),
    static_cast<unsigned>(GetParams()->behavior_tree.groot_server_port),
    GetParams()->behavior_tree.groot_ip_addr.c_str());
  groot_log_file_ =
    std::make_unique<BT::FileLogger>(bt_tree_, GetParams()->behavior_tree.log_fp.c_str());
}

template<typename T>
void RaceDecisionEngineBtBaseLifecycleNode::RegisterTreeNodeTree(const std::string & node_name)
{
  BT::NodeBuilder builder = [&](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, this->GetTtlTree());
    };
  factory_.registerBuilder<T>(node_name, builder);
}

template<typename T>
void RaceDecisionEngineBtBaseLifecycleNode::RegisterTreeNodeLogClock(const std::string & node_name)
{
  BT::NodeBuilder builder = [&](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, this->get_logger(), *this->get_clock());
    };
  factory_.registerBuilder<T>(node_name, builder);
}

template<typename T>
void RaceDecisionEngineBtBaseLifecycleNode::RegisterTreeNodeLogClockTree(
  const std::string & node_name)
{
  BT::NodeBuilder builder = [&](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(
        name, config, this->get_logger(), *this->get_clock(),
        this->GetTtlTree());
    };
  factory_.registerBuilder<T>(node_name, builder);
}

void RaceDecisionEngineBtBaseLifecycleNode::RegisterTreeNodes()
{
  RegisterTreeNodeLogClockTree<bt::action_nodes::PrintValidTtl>("PrintValidTtl");
  RegisterTreeNodeLogClockTree<bt::action_nodes::SetCarState>("SetCarState");
  RegisterTreeNodeLogClockTree<bt::action_nodes::SetClosestRivalCar>("SetClosestRivalCar");
  RegisterTreeNodeLogClockTree<bt::action_nodes::SetSecondClosestRivalCar>(
    "SetSecondClosestRivalCar");
  RegisterTreeNodeLogClockTree<bt::action_nodes::ReduceToTspSpeed>("ReduceToTspSpeed");
  RegisterTreeNodeLogClockTree<bt::action_nodes::SetScaleFactorFromLookup>(
    "SetScaleFactorFromLookup");
  RegisterTreeNodeLogClockTree<bt::action_nodes::FillRivalCarInfo>("FillRivalCarInfo");
  RegisterTreeNodeLogClockTree<bt::action_nodes::FillSecRivalCarInfo>("FillSecRivalCarInfo");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPitBox>("IfInPitBox");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPitCrawl>("IfInPitCrawl");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPitEntrance>("IfInPitEntrance");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPitExit>("IfInPitExit");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPitLane>("IfInPitLane");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPitRoad>("IfInPitRoad");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInStraight>("IfInStraight");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInBackStraight>("IfInBackStraight");
  RegisterTreeNodeLogClockTree<bt::condition_nodes::IfInPassingZoneFromLookup>(
    "IfInPassingZoneFromLookup");

  RegisterTreeNodeLogClock<bt::action_nodes::SetGreenSpeed>("SetGreenSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetYellowSpeed>("SetYellowSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetYellowCatchupSpeed>("SetYellowCatchupSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetRoundSpeed>("SetRoundSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetRoundOvertakingSpeed>("SetRoundOvertakingSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetRoundCatchupSpeed>("SetRoundCatchupSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetPitCrawlSpeed>("SetPitCrawlSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetPitLaneSpeed>("SetPitLaneSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetPitRoadSpeed>("SetPitRoadSpeed");
  RegisterTreeNodeLogClock<bt::action_nodes::SetPitTtlIndex>("SetPitTtlIndex");
  RegisterTreeNodeLogClock<bt::action_nodes::SetRaceTtlIndex>("SetRaceTtlIndex");
  RegisterTreeNodeLogClock<bt::action_nodes::SetLeftTtlIndex>("SetLeftTtlIndex");
  RegisterTreeNodeLogClock<bt::action_nodes::SetRightTtlIndex>("SetRightTtlIndex");
  RegisterTreeNodeLogClock<bt::action_nodes::SetOptimalTtlIndex>("SetOptimalTtlIndex");
  RegisterTreeNodeLogClock<bt::action_nodes::GoJoystickAutonomous>("GoJoystickAutonomous");
  RegisterTreeNodeLogClock<bt::action_nodes::SetJoystickState>("SetJoystickState");
  RegisterTreeNodeLogClock<bt::action_nodes::KeepJoystickState>("KeepJoystickState");
  RegisterTreeNodeLogClock<bt::action_nodes::StopCarEmergency>("StopCarEmergency");
  RegisterTreeNodeLogClock<bt::action_nodes::StopCarImmediate>("StopCarImmediate");
  RegisterTreeNodeLogClock<bt::action_nodes::StopCarSafe>("StopCarSafe");
  RegisterTreeNodeLogClock<bt::action_nodes::IncrementLap>("IncrementLap");
  RegisterTreeNodeLogClock<bt::action_nodes::SetTtlIndex>("SetTtlIndex");
  RegisterTreeNodeLogClock<bt::action_nodes::SetSpeedLimit>("SetSpeedLimit");
  RegisterTreeNodeLogClock<bt::action_nodes::SetToPitting>("SetToPitting");
  RegisterTreeNodeLogClock<bt::action_nodes::SetPush2PassActive>("SetPush2PassActive");
  RegisterTreeNodeLogClock<bt::action_nodes::SetPush2PassInactive>("SetPush2PassInactive");
  RegisterTreeNodeLogClock<bt::action_nodes::ResetScaleFactor>("ResetScaleFactor");
  RegisterTreeNodeLogClock<bt::action_nodes::CreateFalseRivalCar>("CreateFalseRivalCar");
  RegisterTreeNodeLogClock<bt::action_nodes::CreateFalseSecRivalCar>("CreateFalseSecRivalCar");
  RegisterTreeNodeLogClock<bt::action_nodes::SetAllowedToOvertake>("SetAllowedToOvertake");
  RegisterTreeNodeLogClock<bt::action_nodes::SetAttackerOvertakeComplete>(
    "SetAttackerOvertakeComplete");
  RegisterTreeNodeLogClock<bt::action_nodes::SetDefenderOvertakeComplete>(
    "SetDefenderOvertakeComplete");
  RegisterTreeNodeLogClock<bt::action_nodes::SetAttackerOvertakeStarted>(
    "SetAttackerOvertakeStarted");
  RegisterTreeNodeLogClock<bt::action_nodes::SetDefenderOvertakeStarted>(
    "SetDefenderOvertakeStarted");
  RegisterTreeNodeLogClock<bt::action_nodes::SetHighestSpeedRivalCar>("SetHighestSpeedRivalCar");
  RegisterTreeNodeLogClock<bt::action_nodes::SetFollowMode>("SetFollowMode");
  RegisterTreeNodeLogClock<bt::action_nodes::SetNoGap>("SetNoGap");
  RegisterTreeNodeLogClock<bt::action_nodes::SetAttackerPreparingGap>("SetAttackerPreparingGap");
  RegisterTreeNodeLogClock<bt::action_nodes::SetAttackerAttackingGap>("SetAttackerAttackingGap");
  RegisterTreeNodeLogClock<bt::action_nodes::SetDefenderOvertakenGap>("SetDefenderOvertakenGap");
  RegisterTreeNodeLogClock<bt::action_nodes::SetGap>("SetGap");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfInputManualCommandTimeout>(
    "IfInputManualCommandTimeout");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfInputManualCommandEmergency>(
    "IfInputManualCommandEmergency");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfLocalizationTimeout>("IfLocalizationTimeout");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfLocalizationEmergency>("IfLocalizationEmergency");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfRaceControlTimeout>("IfRaceControlTimeout");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfOppCarDetectionTimeout>(
    "IfOppCarDetectionTimeout");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfPurpleFlag>("IfPurpleFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfRedFlag>("IfRedFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfBlackFlag>("IfBlackFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfYellowFlag>("IfYellowFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfGreenFlag>("IfGreenFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfGreenSpeedFlag>("IfGreenSpeedFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfBlueFlag>("IfBlueFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfCheckeredFlag>("IfCheckeredFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfEKillFlag>("IfEkillFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfFcyFlag>("IfFcyFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfNoTrackFlag>("IfNoTrackFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfStopFlag>("IfStopFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfAttackerFlag>("IfAttackerFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfDefenderFlag>("IfDefenderFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfOrangeFlag>("IfOrangeFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfWGreenFlag>("IfWgreenFlag");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfLowLevelFaultSafeStop>("IfLowLevelFaultSafeStop");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfLowLevelFaultImmediateStop>(
    "IfLowLevelFaultImmediateStop");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfLowLevelFaultEmergencyStop>(
    "IfLowLevelFaultEmergencyStop");
  RegisterTreeNodeLogClock<bt::condition_nodes::IfFlagChangedFromWGreen>("IfFlagChangedFromWgreen");

  RegisterTreeNodeLogClock<bt::condition_nodes::IfLowLevelFaultTimeout>("IfLowLevelFaultTimeout");
  RegisterTreeNodeTree<bt::condition_nodes::IfInPitTtl>("IfInPitTtl");
  RegisterTreeNodeTree<bt::condition_nodes::IfInLeftTtl>("IfInLeftTtl");
  RegisterTreeNodeTree<bt::condition_nodes::IfEgoCarLeft>("IfEgoCarLeft");
  RegisterTreeNodeTree<bt::condition_nodes::IfEgoCarNearRaceLine>("IfEgoCarNearRaceLine");
  RegisterTreeNodeTree<bt::condition_nodes::IfEgoCarRight>("IfEgoCarRight");

  factory_.registerNodeType<bt::action_nodes::SetFlags>("SetFlags");
  factory_.registerNodeType<bt::condition_nodes::IfUsePerception>("IfUsePerception");
  factory_.registerNodeType<bt::condition_nodes::IfUseParams>("IfUseParams");
  factory_.registerNodeType<bt::condition_nodes::IfInYellowSpeedRange>("IfInYellowSpeedRange");
  factory_.registerNodeType<bt::condition_nodes::IfInRoundSpeedRange>("IfInRoundSpeedRange");
  factory_.registerNodeType<bt::condition_nodes::IfTspEnabled>("IfTspEnabled");
  factory_.registerNodeType<bt::condition_nodes::IfScaleFactorEnabled>("IfScaleFactorEnabled");
  factory_.registerNodeType<bt::condition_nodes::IfCrossedSf>("IfCrossedSf");
  factory_.registerNodeType<bt::condition_nodes::IfCrossedPassingSf>("IfCrossedPassingSf");
  factory_.registerNodeType<bt::condition_nodes::IfCrossedPassingLine>("IfCrossedPassingLine");
  factory_.registerNodeType<bt::condition_nodes::IfFlagChangedFromGreen>("IfFlagChangedFromGreen");
  factory_.registerNodeType<bt::condition_nodes::IfNoTrackedObjects>("IfNoTrackedObjects");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarBackGapGreaterThan>(
    "IfRivalCarBackGapGreaterThan");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarFrontGapGreaterThan>(
    "IfRivalCarFrontGapGreaterThan");
  factory_.registerNodeType<bt::condition_nodes::IfSecRivalCarFrontGapLesserThan>(
    "IfSecRivalCarFrontGapLesserThan");
  factory_.registerNodeType<bt::condition_nodes::IfSecRivalCarExists>(
    "IfSecRivalCarExists");
  factory_.registerNodeType<bt::condition_nodes::IfSecRivalCarBackGapLesserThan>(
    "IfSecRivalCarBackGapLesserThan");
  factory_.registerNodeType<bt::condition_nodes::IfSecRivalCarBack>(
    "IfSecRivalCarBack");
  factory_.registerNodeType<bt::condition_nodes::IfSecRivalCarFront>(
    "IfSecRivalCarFront");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarBackGapLesserThan>(
    "IfRivalCarBackGapLesserThan");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarFrontGapLesserThan>(
    "IfRivalCarFrontGapLesserThan");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarExists>("IfRivalCarExists");
  factory_.registerNodeType<bt::condition_nodes::IfRivalEgoTtlsMatch>("IfRivalEgoTtlsMatch");
  factory_.registerNodeType<bt::condition_nodes::IfAllowedToOvertake>("IfAllowedToOvertake");
  factory_.registerNodeType<bt::condition_nodes::IfAttackerOvertakeComplete>(
    "IfAttackerOvertakeComplete");
  factory_.registerNodeType<bt::condition_nodes::IfDefenderOvertakeComplete>(
    "IfDefenderOvertakeComplete");
  factory_.registerNodeType<bt::condition_nodes::IfAttackerOvertakeStarted>(
    "IfAttackerOvertakeStarted");
  factory_.registerNodeType<bt::condition_nodes::IfDefenderOvertakeStarted>(
    "IfDefenderOvertakeStarted");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarFront>("IfRivalCarFront");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarBack>("IfRivalCarBack");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarLeft>("IfRivalCarLeft");
  factory_.registerNodeType<bt::condition_nodes::IfRivalCarRight>("IfRivalCarRight");
  factory_.registerNodeType<bt::condition_nodes::IfPitting>("IfPitting");
}

bool RaceDecisionEngineBtBaseLifecycleNode::ProcessInputs(
  const common::RdeOutputs::SharedPtr & output_to_modify)
{
  blackboard_->set<const common::RdeInputs::ConstSharedPtr>("rde_inputs", GetInputs());
  blackboard_->set<const std::shared_ptr<ros_parameters::Params>>("rde_params", GetParams());

  try {
    RunTree();
    output_to_modify->output_manual_cmd_msg->limit_auto_throttle =
      blackboard_->get<bool>("limit_auto_throttle");
    output_to_modify->output_manual_cmd_msg->use_manual_cmd =
      !blackboard_->get<bool>("auto_enabled");
    output_to_modify->ttc_msg->target_speed = blackboard_->get<double>("speed_limit");
    output_to_modify->ttc_msg->target_gap = blackboard_->get<double>("target_gap");
    output_to_modify->ttc_msg->stop_type.stop_type = blackboard_->get<uint8_t>("stop_type");
    output_to_modify->ttc_msg->strategy_type.strategy_type =
      blackboard_->get<uint8_t>("strategy_type");
    auto rival_car = blackboard_->get<common::RivalCar>("current_rival_car");
    output_to_modify->ttc_msg->rival_car_exists = rival_car.exists;
    output_to_modify->ttc_msg->push2pass_cmd = blackboard_->get<bool>("push2pass_cmd");
    if (rival_car.exists) {
      if (std::abs(rival_car.front_gap) < std::abs(rival_car.back_gap)) {
        output_to_modify->ttc_msg->rival_car_gap = rival_car.front_gap;
      } else {
        output_to_modify->ttc_msg->rival_car_gap = -1.0 * rival_car.back_gap;
      }
      output_to_modify->ttc_msg->rival_car_speed = rival_car.speed;
    } else {
      output_to_modify->ttc_msg->rival_car_gap = 0.0;
      output_to_modify->ttc_msg->rival_car_speed = 0.0;
    }
    output_to_modify->ttc_msg->current_ttl_index =
      static_cast<uint8_t>(blackboard_->get<race::ttl::TtlIndex>("output_ttl_index"));
    output_to_modify->ttc_msg->target_waypoint_scale =
      blackboard_->get<double>("scale_factor") / 100.0;
    output_to_modify->rde_telem_msg->x =
      blackboard_->get<const common::CarState>("current_car_state").pos.x;
    output_to_modify->rde_telem_msg->y =
      blackboard_->get<const common::CarState>("current_car_state").pos.y;
    output_to_modify->rde_telem_msg->speed =
      blackboard_->get<const common::CarState>("current_car_state").speed;
    output_to_modify->ttc_msg->in_pit_ttl =
      blackboard_->get<std::string>("selected_ttl") == "pit";
    output_to_modify->rde_telem_msg->is_pose_accurate =
      GetInputs()->localization_state_msg->source_status_code > 0;
    output_to_modify->rde_telem_msg->track_flag = static_cast<uint8_t>(
      race::get_track_flag_from_flags(blackboard_->get<const race::Flags>("current_flags")));
    output_to_modify->rde_telem_msg->vehicle_flag = static_cast<uint8_t>(
      race::get_vehicle_flag_from_flags(blackboard_->get<const race::Flags>("current_flags")));
    output_to_modify->rde_telem_msg->defender_target_speed =
      GetInputs()->race_control_msg->round_target_speed;
    output_to_modify->rde_telem_msg->emergency_stop_cmd =
      GetInputs()->input_manual_cmd_msg->vehicle_control_command.emergency_stop_cmd;
    output_to_modify->output_manual_cmd_msg->vehicle_control_command.push2pass_cmd =
      GetInputs()->input_manual_cmd_msg->vehicle_control_command.push2pass_cmd;
    output_to_modify->rde_telem_msg->ttl_index = static_cast<uint8_t>(
      blackboard_->get<const common::CarState>("current_car_state").current_ttl_index);
    output_to_modify->rde_telem_msg->track_location = static_cast<uint8_t>(
      GetTtlTree()
      ->get_ttl(
        blackboard_->get<const common::CarState>("current_car_state").current_ttl_index)
      .waypoints
      .at(blackboard_->get<const common::CarState>("current_car_state").current_ttl_wp_index)
      .region);
    output_to_modify->rde_telem_msg->distance_from_sf =
      blackboard_->get<const common::CarState>("current_car_state").lap_distance;
    output_to_modify->rde_telem_msg->lap_percentage =
      blackboard_->get<const common::CarState>("current_car_state").lap_percentage;
    output_to_modify->rde_telem_msg->waypoint_index =
      blackboard_->get<const common::CarState>("current_car_state").current_ttl_wp_index;
    output_to_modify->rde_telem_msg->num_laps = blackboard_->get<double>("num_laps");
    if (GetParams()->target_speed_profiles.tsp_segment_names.size() >
      blackboard_->get<size_t>("tsp_index"))
    {
      output_to_modify->rde_telem_msg->tsp_telemetry.current_tsp_segment =
        GetParams()
        ->target_speed_profiles.tsp_segment_names[blackboard_->get<size_t>("tsp_index")];
    } else {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(),
        GetParams()->non_critical_log_time_period_ms,
        "TSP index out of bounds");
    }
    PostRunTree();
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(this->get_logger(), ex.what());
  }

  return false;
}

void RaceDecisionEngineBtBaseLifecycleNode::PostRunTree() {}

BT::Blackboard::Ptr & RaceDecisionEngineBtBaseLifecycleNode::GetBlackboard() {return blackboard_;}

BT::Tree & RaceDecisionEngineBtBaseLifecycleNode::GetBtTree() {return bt_tree_;}
}  // namespace base
}  // namespace nodes
}  // namespace race_decision_engine
}  // namespace planning
}  // namespace race
